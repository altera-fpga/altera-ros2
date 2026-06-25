// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#include <cmath>
#include <map>
#include <signal.h>
#include <string>
#include <thread>

#include "marker_controller.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <rclcpp/rclcpp.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>

#include <xarm_msgs/srv/call.hpp>
#include <xarm_msgs/srv/set_int16.hpp>
#include <xarm_msgs/srv/set_int16_by_id.hpp>
#include <xarm_msgs/srv/move_joint.hpp>

#include <aruco_markers_msgs/msg/marker.hpp>
#include <aruco_markers_msgs/msg/marker_array.hpp>



#define SERVICE_CALL_FAILED 999


std::shared_ptr<rclcpp::Node> node;

std::shared_ptr<xarm_msgs::srv::Call::Request> req = std::make_shared<xarm_msgs::srv::Call::Request>();
std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req_int16 = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req_int16_by_id = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
std::shared_ptr<xarm_msgs::srv::MoveJoint::Request> req_move_joint = std::make_shared<xarm_msgs::srv::MoveJoint::Request>();

rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_open_gripper;
rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_close_gripper;
rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_stop_gripper;

double block_detect_y = 0.0;
int block_id = -1;
std::map<int, geometry_msgs::msg::Pose> block_poses;
std::map<int, geometry_msgs::msg::Pose> pose_buffer;

std::vector<moveit_msgs::msg::CollisionObject> block_objects;
std::vector<moveit_msgs::msg::ObjectColor> block_colors;

std::unique_ptr<tf2_ros::Buffer> tf_buffer;
std::shared_ptr<tf2_ros::TransformListener> tf_listener;
geometry_msgs::msg::TransformStamped camera_to_base;

std_msgs::msg::ColorRGBA dark_blue_color;
std_msgs::msg::ColorRGBA khaki_color;
std_msgs::msg::ColorRGBA object_color;

bool marker_detection = false;

double camera_offset_z = 0.0;
double camera_offset_x = 0.0;
double camera_offset_y = 0.0;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_block_camera_demo] shutting down...\n");
    exit(-1);
}

template<typename ServiceT, typename SharedRequest>
int call_request(std::shared_ptr<ServiceT> client, SharedRequest request)
{
    bool is_try_again = false;
    while (!client->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
            RCLCPP_ERROR(node->get_logger(), "Interrupted while waiting for the service. Exiting.");
            exit(1);
        }
        if (!is_try_again) {
            is_try_again = true;
            RCLCPP_WARN(node->get_logger(), "service %s not available, waiting ...", client->get_service_name());
        }
    }
    auto result_future = client->async_send_request(request);
    auto res = result_future.get();
    if (res->message.size() != 0)
        RCLCPP_DEBUG(node->get_logger(), "call service %s, ret=%d, message(%s)", client->get_service_name(), res->ret, res->message.c_str());
    else
        RCLCPP_DEBUG(node->get_logger(), "call service %s, ret=%d", client->get_service_name(), res->ret);
    return res->ret;
}

void open_gripper()
{
    auto future = std::async(std::launch::async,
        []()
        {
            call_request(client_open_gripper, req);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            call_request(client_stop_gripper, req);
        });

    future.get();
}

void close_gripper()
{
    auto future = std::async(std::launch::async,
        []()
        {
            call_request(client_close_gripper, req);
            std::this_thread::sleep_for(std::chrono::seconds(1));
            call_request(client_stop_gripper, req);
        });

    future.get();
}

void marker_callback(const aruco_markers_msgs::msg::MarkerArray & msg)
{
    if (!marker_detection) return;

    camera_to_base = tf_buffer->lookupTransform("link_base", "xarm_camera_color_optical_frame", tf2::TimePointZero);

    // Clear existing block poses
    block_poses.clear();
    block_objects.clear();
    block_colors.clear();

    bool accurate_markers = true;

    for (aruco_markers_msgs::msg::Marker marker : msg.markers) {
        if (marker.id > 9) continue;

        RCLCPP_INFO_STREAM(node->get_logger(), "Marker Pose:" << geometry_msgs::msg::to_yaml(marker.pose));

        geometry_msgs::msg::PoseStamped pose;
        tf2::doTransform(marker.pose, pose, camera_to_base);

        RCLCPP_INFO_STREAM(node->get_logger(), "Transformed Pose:" << geometry_msgs::msg::to_yaml(pose.pose));

        // Make sure marker is in front of the robot arm
        if (pose.pose.position.x < 0.20) {
            continue;
        }

        // Get marker pose from buffer
        auto it = pose_buffer.find(marker.id);

        if (it == pose_buffer.end()) {
            pose_buffer[marker.id] = pose.pose;
            accurate_markers = false;
            continue;
        } else {
            // Check if marker pose has changed significantly
            double distance = std::abs(pose.pose.position.x - it->second.position.x) +
                              std::abs(pose.pose.position.y - it->second.position.y) +
                              std::abs(pose.pose.position.z - it->second.position.z);

            if (distance > 0.005) {
                pose_buffer[marker.id] = pose.pose;
                accurate_markers = false;
                continue;
            }
        }

        // Calculate height of block based on camera offset
        pose.pose.position.z = std::max(pose.pose.position.z + camera_offset_z - 0.0125, 0.0125);
        pose.pose.position.x = pose.pose.position.x + camera_offset_x;
        pose.pose.position.y = pose.pose.position.y + camera_offset_y;
        pose.pose.orientation.y = marker.pose.pose.orientation.x;
        pose.pose.orientation.x = marker.pose.pose.orientation.y;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 0.0;

        RCLCPP_INFO_STREAM(node->get_logger(), "Corrected Pose:" << geometry_msgs::msg::to_yaml(pose.pose));

        block_poses[marker.id] = pose.pose;

        // Planning scene
        moveit_msgs::msg::CollisionObject block_object;
        block_object.header.frame_id = "world";
        block_object.id = "block" + std::to_string(marker.id);
        shape_msgs::msg::SolidPrimitive block_primitive;
        block_primitive.type = block_primitive.BOX;
        block_primitive.dimensions.resize(3);
        block_primitive.dimensions[block_primitive.BOX_X] = 0.025;
        block_primitive.dimensions[block_primitive.BOX_Y] = 0.025;
        block_primitive.dimensions[block_primitive.BOX_Z] = 0.025;
        block_object.primitives.push_back(block_primitive);
        block_object.primitive_poses.push_back(pose.pose);
        block_object.operation = block_object.ADD;
        moveit_msgs::msg::ObjectColor block_color;
        block_color.id = "block" + std::to_string(marker.id);
        block_color.color = dark_blue_color;

        block_objects.push_back(block_object);
        block_colors.push_back(block_color);
    }

    // If all markers are accurately detected stop detection
    if (accurate_markers) {
        marker_detection = false;
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node = rclcpp::Node::make_shared("xarm_block_camera_demo", node_options);
    RCLCPP_INFO(node->get_logger(), "xarm_block_camera_demo start");
    signal(SIGINT, exit_sig_handler);

    tf_buffer = std::make_unique<tf2_ros::Buffer>(node->get_clock());
    tf_listener = std::make_shared<tf2_ros::TransformListener>(*tf_buffer);

    client_open_gripper = node->create_client<xarm_msgs::srv::Call>("xarm/open_lite6_gripper");
    client_close_gripper = node->create_client<xarm_msgs::srv::Call>("xarm/close_lite6_gripper");
    client_stop_gripper = node->create_client<xarm_msgs::srv::Call>("xarm/stop_lite6_gripper");

    auto planning_group = node->declare_parameter("planning_group", "lite6");
    bool use_marker_generator = node->declare_parameter("use_marker_generator", true);
    auto mode = node->declare_parameter("mode", "fake");
    camera_offset_z = node->declare_parameter("offset_z", 0.0);
    camera_offset_x = node->declare_parameter("offset_x", 0.0);
    camera_offset_y = node->declare_parameter("offset_y", 0.0);

    // Initialize marker generator if enabled
    MarkerController marker_controller;

    if (use_marker_generator) {
        marker_controller.open();
        marker_controller.remix();
    }

    moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);

    if (mode == "real") {
        move_group.setMaxVelocityScalingFactor(0.5);
    } else {
        move_group.setMaxVelocityScalingFactor(1.0);
    }

    move_group.allowReplanning(true);
    move_group.setPlanningTime(1.0);

    object_color.r = 1.0;
    object_color.g = 1.0;
    object_color.b = 1.0;
    object_color.a = 0.99;

    khaki_color.r = 0.94;
    khaki_color.g = 0.9;
    khaki_color.b = 0.55;
    khaki_color.a = 1.0;

    dark_blue_color.r = 0.0;
    dark_blue_color.g = 0.1176;
    dark_blue_color.b = 0.314;
    dark_blue_color.a = 1.0;

    // Setup scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    std::vector<moveit_msgs::msg::ObjectColor> object_colors;

    // Remove existing items from planning scene
    for (auto& o : planning_scene_interface.getObjects()) {
        o.second.operation = o.second.REMOVE;
        planning_scene_interface.applyCollisionObject(o.second);
    }

    moveit_msgs::msg::CollisionObject back_object;
    back_object.header.frame_id = "world";
    back_object.id = "back";
    shape_msgs::msg::SolidPrimitive back_primitive;
    back_primitive.type = back_primitive.BOX;
    back_primitive.dimensions.resize(3);
    back_primitive.dimensions[back_primitive.BOX_X] = 0.1;
    back_primitive.dimensions[back_primitive.BOX_Y] = 1.0;
    back_primitive.dimensions[back_primitive.BOX_Z] = 1.0;
    geometry_msgs::msg::Pose back_pose;
    back_pose.position.x = -0.25;
    back_pose.position.y = 0.0;
    back_pose.position.z = 0.5;
    back_object.primitives.push_back(back_primitive);
    back_object.primitive_poses.push_back(back_pose);
    back_object.operation = back_object.ADD;
    moveit_msgs::msg::ObjectColor back_color;
    back_color.id = "back";
    back_color.color = object_color;

    moveit_msgs::msg::CollisionObject table_object;
    table_object.header.frame_id = "world";
    table_object.id = "table";
    shape_msgs::msg::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[table_primitive.BOX_X] = 0.6;
    table_primitive.dimensions[table_primitive.BOX_Y] = 1.0;
    table_primitive.dimensions[table_primitive.BOX_Z] = 0.1;
    geometry_msgs::msg::Pose table_pose;
    table_pose.position.x = 0.10;
    table_pose.position.y = 0.0;
    table_pose.position.z = -0.05;
    table_object.primitives.push_back(table_primitive);
    table_object.primitive_poses.push_back(table_pose);
    table_object.operation = table_object.ADD;
    moveit_msgs::msg::ObjectColor table_color;
    table_color.id = "table";
    table_color.color = object_color;

    moveit_msgs::msg::CollisionObject box_object;
    box_object.header.frame_id = "world";
    box_object.id = "box";
    shape_msgs::msg::SolidPrimitive box_primitive;
    box_primitive.type = box_primitive.BOX;
    box_primitive.dimensions.resize(3);
    box_primitive.dimensions[box_primitive.BOX_X] = 0.1;
    box_primitive.dimensions[box_primitive.BOX_Y] = 0.1;
    box_primitive.dimensions[box_primitive.BOX_Z] = 0.07;
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = 0.0;
    box_pose.position.y = -0.15;
    box_pose.position.z = 0.04;
    box_object.primitives.push_back(box_primitive);
    box_object.primitive_poses.push_back(box_pose);
    box_object.operation = box_object.ADD;
    moveit_msgs::msg::ObjectColor box_color;
    box_color.id = "box";
    box_color.color = khaki_color;

    geometry_msgs::msg::Pose box_hover_pose;
    box_hover_pose.position.x = 0.0;
    box_hover_pose.position.y = -0.15;
    box_hover_pose.position.z = 0.15;
    box_hover_pose.orientation.x = 1.0;
    box_hover_pose.orientation.y = 0.0;
    box_hover_pose.orientation.z = 0.0;
    box_hover_pose.orientation.w = 0.0;

    collision_objects.push_back(back_object);
    collision_objects.push_back(table_object);
    collision_objects.push_back(box_object);

    object_colors.push_back(back_color);
    object_colors.push_back(table_color);
    object_colors.push_back(box_color);

    // Block Detection Pose
    geometry_msgs::msg::Pose block_detect_pose;
    block_detect_pose.position.x = 0.2;
    block_detect_pose.position.y = 0.0;
    block_detect_pose.position.z = 0.4;
    block_detect_pose.orientation.x = 1.0;
    block_detect_pose.orientation.y = 0.0;
    block_detect_pose.orientation.z = 0.0;
    block_detect_pose.orientation.w = 0.0;

    planning_scene_interface.addCollisionObjects(collision_objects, object_colors);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::msg::RobotTrajectory trajectory;
    bool success = false;

    // Marker Subscriber
    rclcpp::Subscription<aruco_markers_msgs::msg::MarkerArray>::SharedPtr marker_subscriber = node->create_subscription<aruco_markers_msgs::msg::MarkerArray>(
            "/aruco/markers", 1, marker_callback);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();


    //
    // Main Loop
    //

    if (mode == "real") {
        open_gripper();
    }

    while (rclcpp::ok()) {
        success = false;

        while (!success) {
            move_group.setStartStateToCurrentState();
            block_detect_pose.position.y = block_detect_y;
            move_group.setPoseTarget(block_detect_pose);

            if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                continue;
            }

            success = true;
        }

        // Enable marker detection
        marker_detection = true;

        int d_count = 0;

        while (marker_detection) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
            d_count++;
            
            // Remix markers if no blocks are detected after 5 seconds
            if (d_count == 5 && use_marker_generator) {
                marker_controller.remix();
                d_count = 0;
            }
        }
        
        // Update planning scene
        planning_scene_interface.applyCollisionObjects(block_objects, block_colors);

        // Choose block to pick
        geometry_msgs::msg::Pose pose;

        if (block_id < 0) {
            block_id = block_poses.begin()->first;
            pose = block_poses.begin()->second;
        } else {
            auto it = block_poses.find(block_id);
            if (it != block_poses.end()) {
                pose = it->second;
            } else {
                block_id = -1;
                block_detect_y = 0.0;
                continue;
            }
        }

        // Check alignment of end effector with block if using real camera
        if (!use_marker_generator) {
            geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
            double pose_diff = std::abs(current_pose.pose.position.y - pose.position.y);

            if (block_id < 0 || pose_diff > 0.02) {
                block_detect_y = pose.position.y;
                continue;
            }
        }

        geometry_msgs::msg::Pose pose_hover = pose;
        pose_hover.position.z += 0.03;

        geometry_msgs::msg::Pose pose_grab = pose;

        std::vector<geometry_msgs::msg::Pose> down_waypoints;
        down_waypoints.push_back(pose_hover);
        down_waypoints.push_back(pose_grab);

        std::vector<geometry_msgs::msg::Pose> up_waypoints;
        up_waypoints.push_back(pose_grab);
        up_waypoints.push_back(pose_hover);

        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(pose_hover);
        
        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            // Remix markers if execution failed to avoid a failure loop
            if (use_marker_generator) {
                marker_controller.remix();
            }

            // Reset y position for next detection
            block_detect_y = 0.0;
            block_id = -1;

            // Remove blocks from planning scene
            for (moveit_msgs::msg::CollisionObject o : block_objects) {
                o.operation = o.REMOVE;
                planning_scene_interface.applyCollisionObject(o);
            }

            continue;
        }

        if (mode == "real") {
            open_gripper();
        }

        move_group.setStartStateToCurrentState();
        move_group.computeCartesianPath(down_waypoints, 0.01, trajectory, false);

        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
            // Reset y position for next detection
            block_detect_y = 0.0;
            block_id = -1;

            // Remove blocks from planning scene
            for (moveit_msgs::msg::CollisionObject o : block_objects) {
                o.operation = o.REMOVE;
                planning_scene_interface.applyCollisionObject(o);
            }
        }

        if (mode == "real") {
            close_gripper();
        }

        // Attach block
        std::vector<std::string> touch_links = {move_group.getEndEffectorLink(), "table"};
        move_group.attachObject("block" + std::to_string(block_id), move_group.getEndEffectorLink(), touch_links);

        if (use_marker_generator) {
            marker_controller.hide_marker(block_id);
        }

        move_group.setStartStateToCurrentState();
        move_group.computeCartesianPath(up_waypoints, 0.01, trajectory, false);

        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
            if (mode == "real") {
                open_gripper();
            }

            // Detach block
            move_group.detachObject(move_group.getEndEffectorLink());

            // Reset y position for next detection
            block_detect_y = 0.0;
            block_id = -1;

            // Remove blocks from planning scene
            for (moveit_msgs::msg::CollisionObject o : block_objects) {
                o.operation = o.REMOVE;
                planning_scene_interface.applyCollisionObject(o);
            }

            continue;
        }

        move_group.setStartStateToCurrentState();
        move_group.setPoseTarget(box_hover_pose);

        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            if (mode == "real") {
                open_gripper();
            }

            // Detach block
            move_group.detachObject(move_group.getEndEffectorLink());

            // Reset y position for next detection
            block_detect_y = 0.0;
            block_id = -1;

            // Remove blocks from planning scene
            for (moveit_msgs::msg::CollisionObject o : block_objects) {
                o.operation = o.REMOVE;
                planning_scene_interface.applyCollisionObject(o);
            }

            continue;
        }

        if (mode == "real") {
            open_gripper();
        }

        // Detach block
        move_group.detachObject(move_group.getEndEffectorLink());

        // Reset y position for next detection
        block_detect_y = 0.0;
        block_id = -1;

        // Remove blocks from planning scene
        while (planning_scene_interface.getObjects().size() > 3) {
            for (moveit_msgs::msg::CollisionObject o : block_objects) {
                o.operation = o.REMOVE;
                planning_scene_interface.applyCollisionObject(o);
            }
        }
    }

    if (use_marker_generator) {
        marker_controller.close();
    }

    RCLCPP_INFO(node->get_logger(), "xarm_block_demo finished");
    return 0;
}

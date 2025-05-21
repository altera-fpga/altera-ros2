// Copyright (C) 2025 Altera Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <signal.h>
#include <string>
#include <thread>

#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <geometry_msgs/msg/pose.hpp>
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
#include <xarm_msgs/srv/move_home.hpp>
#include <xarm_msgs/srv/move_joint.hpp>


#define SERVICE_CALL_FAILED 999

std::shared_ptr<rclcpp::Node> node;

std::shared_ptr<xarm_msgs::srv::Call::Request> req = std::make_shared<xarm_msgs::srv::Call::Request>();
std::shared_ptr<xarm_msgs::srv::SetInt16::Request> req_int16 = std::make_shared<xarm_msgs::srv::SetInt16::Request>();
std::shared_ptr<xarm_msgs::srv::SetInt16ById::Request> req_int16_by_id = std::make_shared<xarm_msgs::srv::SetInt16ById::Request>();
std::shared_ptr<xarm_msgs::srv::MoveHome::Request> req_move_home = std::make_shared<xarm_msgs::srv::MoveHome::Request>();
std::shared_ptr<xarm_msgs::srv::MoveJoint::Request> req_move_joint = std::make_shared<xarm_msgs::srv::MoveJoint::Request>();

rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_open_gripper;
rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_close_gripper;
rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_stop_gripper;
rclcpp::Client<xarm_msgs::srv::Call>::SharedPtr client_clean_error;
rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_mode;
rclcpp::Client<xarm_msgs::srv::SetInt16>::SharedPtr client_set_state;
rclcpp::Client<xarm_msgs::srv::SetInt16ById>::SharedPtr client_motion_enable;
rclcpp::Client<xarm_msgs::srv::MoveHome>::SharedPtr client_move_home;
rclcpp::Client<xarm_msgs::srv::MoveJoint>::SharedPtr client_set_servo_angle;

void exit_sig_handler(int signum)
{
    fprintf(stderr, "[xarm_block_demo] shutting down...\n");
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

void clear_error()
{
    RCLCPP_INFO(node->get_logger(), "Clearing error...");

    // Clear error
    call_request(client_clean_error, req);

    // Enable motion
    req_int16_by_id->id = 8;
    req_int16_by_id->data = (int)true;
    call_request(client_motion_enable, req_int16_by_id);

    // Set state
    req_int16->data = 0;
    call_request(client_set_state, req_int16);

    // Move home
    req_move_home->speed = 0;
    req_move_home->acc = 0;
    req_move_home->mvtime = 0;
    req_move_home->wait = true;
    req_move_home->timeout = 5;
    call_request(client_move_home, req_move_home);

    // Set Mode
    req_int16->data = 1;
    call_request(client_set_mode, req_int16);

    // Set state
    req_int16->data = 0;
    call_request(client_set_state, req_int16);
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

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node = rclcpp::Node::make_shared("xarm_block_demo", node_options);
    RCLCPP_INFO(node->get_logger(), "xarm_block_demo start");
    signal(SIGINT, exit_sig_handler);

    client_open_gripper = node->create_client<xarm_msgs::srv::Call>("xarm/open_lite6_gripper");
    client_close_gripper = node->create_client<xarm_msgs::srv::Call>("xarm/close_lite6_gripper");
    client_stop_gripper = node->create_client<xarm_msgs::srv::Call>("xarm/stop_lite6_gripper");
    client_clean_error = node->create_client<xarm_msgs::srv::Call>("xarm/clean_error");
    client_set_mode = node->create_client<xarm_msgs::srv::SetInt16>("xarm/set_mode");
    client_set_state = node->create_client<xarm_msgs::srv::SetInt16>("xarm/set_state");
    client_motion_enable = node->create_client<xarm_msgs::srv::SetInt16ById>("xarm/motion_enable");
    client_move_home = node->create_client<xarm_msgs::srv::MoveHome>("xarm/move_gohome");
    client_set_servo_angle = node->create_client<xarm_msgs::srv::MoveJoint>("xarm/set_servo_angle_j");

    auto planning_group = node->get_parameter("planning_group");
    auto planning_group_2 = node->get_parameter("planning_group_2");
    auto mode_param = node->get_parameter("mode");
    auto prefix_param = node->get_parameter("prefix");
    auto offset_param = node->get_parameter("offset");

    std::string pg1 = planning_group.as_string();
    std::string pg2 = planning_group_2.as_string();
    std::string mode = mode_param.as_string();
    std::string prefix = prefix_param.as_string();
    double y_offset = offset_param.as_double();

    int planning_groups = 1;

    if (pg2.empty()) {
        pg2 = pg1;
    } else {
        planning_groups = 2;
    }

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    moveit::planning_interface::MoveGroupInterface move_group(node, pg1);
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.allowReplanning(true);
    move_group.setReplanAttempts(5);
    move_group.setReplanDelay(1);
    move_group.setPlanningTime(10);

    moveit::planning_interface::MoveGroupInterface move_group_2(node, pg2);
    move_group_2.setMaxVelocityScalingFactor(0.5);
    move_group_2.allowReplanning(true);
    move_group_2.setReplanAttempts(5);
    move_group_2.setReplanDelay(1);
    move_group_2.setPlanningTime(10);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    std_msgs::msg::ColorRGBA object_color;
    object_color.r = 1.0;
    object_color.g = 1.0;
    object_color.b = 1.0;
    object_color.a = 0.99;

    std_msgs::msg::ColorRGBA dark_blue_color;
    dark_blue_color.r = 0.0;
    dark_blue_color.g = 0.1176;
    dark_blue_color.b = 0.314;
    dark_blue_color.a = 1.0;

    std_msgs::msg::ColorRGBA light_blue_color;
    light_blue_color.r = 0.0;
    light_blue_color.g = 0.78;
    light_blue_color.b = 0.99;
    light_blue_color.a = 1.0;

    // Setup scene
    std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
    std::vector<moveit_msgs::msg::ObjectColor> object_colors;

    std::array<float, 2> offsets{{0.0, y_offset}};

    for (int i = 0; i < planning_groups; i++) {
        moveit_msgs::msg::CollisionObject back_object;
        back_object.header.frame_id = "world";
        back_object.id = "back_" + std::to_string(i);
        shape_msgs::msg::SolidPrimitive back_primitive;
        back_primitive.type = back_primitive.BOX;
        back_primitive.dimensions.resize(3);
        back_primitive.dimensions[back_primitive.BOX_X] = 0.1;
        back_primitive.dimensions[back_primitive.BOX_Y] = 1.0;
        back_primitive.dimensions[back_primitive.BOX_Z] = 1.0;
        geometry_msgs::msg::Pose back_pose;
        back_pose.position.x = -0.25;
        back_pose.position.y = 0.0 + offsets[i];
        back_pose.position.z = 0.5;
        back_object.primitives.push_back(back_primitive);
        back_object.primitive_poses.push_back(back_pose);
        back_object.operation = back_object.ADD;
        moveit_msgs::msg::ObjectColor back_color;
        back_color.id = "back_" + std::to_string(i);
        back_color.color = object_color;

        moveit_msgs::msg::CollisionObject table_object;
        table_object.header.frame_id = "world";
        table_object.id = "table_" + std::to_string(i);
        shape_msgs::msg::SolidPrimitive table_primitive;
        table_primitive.type = table_primitive.BOX;
        table_primitive.dimensions.resize(3);
        table_primitive.dimensions[table_primitive.BOX_X] = 0.6;
        table_primitive.dimensions[table_primitive.BOX_Y] = 1.0;
        table_primitive.dimensions[table_primitive.BOX_Z] = 0.1;
        geometry_msgs::msg::Pose table_pose;
        table_pose.position.x = 0.10;
        table_pose.position.y = 0.0 + offsets[i];
        table_pose.position.z = -0.05;
        table_object.primitives.push_back(table_primitive);
        table_object.primitive_poses.push_back(table_pose);
        table_object.operation = table_object.ADD;
        moveit_msgs::msg::ObjectColor table_color;
        table_color.id = "table_" + std::to_string(i);
        table_color.color = object_color;

        collision_objects.push_back(back_object);
        collision_objects.push_back(table_object);

        object_colors.push_back(back_color);
        object_colors.push_back(table_color);
    }

    std::vector<moveit_msgs::msg::CollisionObject> block_objects;
    std::vector<moveit_msgs::msg::ObjectColor> block_colors;
    std::vector<geometry_msgs::msg::Pose> block_poses;

    std::array<float, 10> block_x_positions{0.13, 0.13, 0.13, 0.13, 0.13, 0.13, 0.13, 0.13, 0.13, 0.13};
    std::array<float, 10> block_y_positions{-0.1818, -0.1414, -0.1010, -0.0606, -0.0202, 0.0202, 0.0606, 0.1010, 0.1414, 0.1818};

    for (int i = 0; i < 10; i++) {
        geometry_msgs::msg::Pose block_pose;
        block_pose.position.x = block_x_positions[i];
        block_pose.position.y = block_y_positions[i];
        block_pose.position.z = 0.01;
        block_pose.orientation.x = 1.0;
        block_pose.orientation.y = 0.0;
        block_pose.orientation.z = 0.0;
        block_pose.orientation.w = 0.0;

        block_poses.push_back(block_pose);
    }

    for (int i = 0; i < planning_groups; i++) {
        for (int j = 0; j < 10; j++) {
            moveit_msgs::msg::CollisionObject block_object;
            block_object.header.frame_id = "world";
            block_object.id = "block" + std::to_string(j) + "_" + std::to_string(i);
            shape_msgs::msg::SolidPrimitive block_primitive;
            block_primitive.type = block_primitive.BOX;
            block_primitive.dimensions.resize(3);
            block_primitive.dimensions[block_primitive.BOX_X] = 0.025;
            block_primitive.dimensions[block_primitive.BOX_Y] = 0.025;
            block_primitive.dimensions[block_primitive.BOX_Z] = 0.025;
            geometry_msgs::msg::Pose block_pose;
            block_pose.position.x = block_x_positions[j];
            block_pose.position.y = offsets[i] + block_y_positions[j];
            block_pose.position.z = 0.0130;
            block_object.primitives.push_back(block_primitive);
            block_object.primitive_poses.push_back(block_pose);
            block_object.operation = block_object.ADD;
            moveit_msgs::msg::ObjectColor block_color;
            block_color.id = "block" + std::to_string(j) + "_" + std::to_string(i);

            if (j > 0) {
                block_color.color = dark_blue_color;
            } else {
                block_color.color = light_blue_color;
            }

            block_objects.push_back(block_object);
            block_colors.push_back(block_color);
        }
    }

    planning_scene_interface.addCollisionObjects(collision_objects, object_colors);
    planning_scene_interface.addCollisionObjects(block_objects, block_colors);

    std::array<float, 8> face_x_positions{0.30, 0.24, 0.24, 0.38, 0.36, 0.36, 0.325, 0.325};
    std::array<float, 8> face_y_positions{0.0, 0.04, -0.04, 0.0, 0.04, -0.04, 0.075, -0.075};
    std::array<float, 8> face_z_positions{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

    std::array<float, 10> logo_x_positions{0.2375, 0.2375, 0.2375, 0.2929, 0.3333, 0.3737, 0.3662, 0.3662, 0.32205, 0.28165};
    std::array<float, 10> logo_y_positions{0.0756, 0.0202, -0.0202, 0.0756, 0.0756, 0.0756, 0.0202, -0.0202, -0.0606, -0.0606};
    std::array<float, 10> logo_z_positions{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

    std::array<float, 10> tower_x_positions{0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30, 0.30};
    std::array<float, 10> tower_y_positions{0.0606, 0.0202, -0.0202, -0.0606, 0.0, 0.0404, -0.0404, 0.0202, -0.0202, 0.0};
    std::array<float, 10> tower_z_positions{0.01, 0.01, 0.01, 0.01, 0.0354, 0.0354, 0.0354, 0.0608, 0.0608, 0.0862};

    std::array<float, 10> five_x_positions{0.2075, 0.2075, 0.2075, 0.2479, 0.2883, 0.2883, 0.3137, 0.3541, 0.3795, 0.3795};
    std::array<float, 10> five_y_positions{0.0404, 0.0, -0.0404, -0.0404, -0.0404, 0.0, 0.0404, 0.0404, 0.0, -0.0404};
    std::array<float, 10> five_z_positions{0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01};

    std::vector<std::vector<geometry_msgs::msg::Pose>> pattern_poses;
    std::vector<bool> pattern_random;

    // Altera Logo
    std::vector<geometry_msgs::msg::Pose> logo_poses;
    for (int i = 0; i < 10; i++) {
        geometry_msgs::msg::Pose block_pose;
        block_pose.orientation.x = 1.0;
        block_pose.orientation.y = 0.0;
        block_pose.orientation.z = 0.0;
        block_pose.orientation.w = 0.0;
        block_pose.position.x = logo_x_positions[i];
        block_pose.position.y = logo_y_positions[i];
        block_pose.position.z = logo_z_positions[i];

        logo_poses.push_back(block_pose);
    }

    // Smiley Face
    std::vector<geometry_msgs::msg::Pose> face_poses;
    for (int i = 0; i < 8; i++) {
        geometry_msgs::msg::Pose block_pose;
        block_pose.orientation.x = 1.0;
        block_pose.orientation.y = 0.0;
        block_pose.orientation.z = 0.0;
        block_pose.orientation.w = 0.0;
        block_pose.position.x = face_x_positions[i];
        block_pose.position.y = face_y_positions[i];
        block_pose.position.z = face_z_positions[i];

        face_poses.push_back(block_pose);
    }

    // Tower
    std::vector<geometry_msgs::msg::Pose> tower_poses;
    for (int i = 0; i < 10; i++) {
        geometry_msgs::msg::Pose block_pose;
        block_pose.orientation.x = 1.0;
        block_pose.orientation.y = 0.0;
        block_pose.orientation.z = 0.0;
        block_pose.orientation.w = 0.0;
        block_pose.position.x = tower_x_positions[i];
        block_pose.position.y = tower_y_positions[i];
        block_pose.position.z = tower_z_positions[i];

        tower_poses.push_back(block_pose);
    }

    // Five
    std::vector<geometry_msgs::msg::Pose> five_poses;
    for (int i = 0; i < 10; i++) {
        geometry_msgs::msg::Pose block_pose;
        block_pose.orientation.x = 1.0;
        block_pose.orientation.y = 0.0;
        block_pose.orientation.z = 0.0;
        block_pose.orientation.w = 0.0;
        block_pose.position.x = five_x_positions[i];
        block_pose.position.y = five_y_positions[i];
        block_pose.position.z = five_z_positions[i];

        five_poses.push_back(block_pose);
    }

    pattern_poses.push_back(logo_poses);
    pattern_random.push_back(true);
    pattern_poses.push_back(face_poses);
    pattern_random.push_back(true);
    pattern_poses.push_back(tower_poses);
    pattern_random.push_back(false);
    pattern_poses.push_back(five_poses);
    pattern_random.push_back(true);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit_msgs::msg::RobotTrajectory trajectory;
    bool success = false;
    bool gripper_twist = false;

    // Move home
    while (!success) {
        move_group.setStartStateToCurrentState();
        move_group.setNamedTarget("home");
        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
            if (mode == "real") {
                clear_error();
            }
            continue;
        }

        success = true;
    }

    if (planning_groups > 1) {
        while (!success) {
            move_group_2.setStartStateToCurrentState();
            move_group_2.setNamedTarget("home");
            if (move_group_2.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                continue;
            }

            success = true;
        }
    }

    // Setup to align blocks
    if (mode == "real") {
        for (int i = 0; i < 10; i++) {
            geometry_msgs::msg::Pose pose_hover = block_poses[i];
            pose_hover.position.z = 0.1;

            geometry_msgs::msg::Pose pose = block_poses[i];

            pose_hover.orientation.x = 0.07;
            pose_hover.orientation.y = 0.07;
            pose.orientation.x = 0.07;
            pose.orientation.y = 0.07;

            std::vector<geometry_msgs::msg::Pose> down_waypoints;
            down_waypoints.push_back(pose_hover);
            down_waypoints.push_back(pose);

            std::vector<geometry_msgs::msg::Pose> up_waypoints;
            up_waypoints.push_back(pose);
            up_waypoints.push_back(pose_hover);

            success = false;
            while (!success) {
                move_group.setStartStateToCurrentState();
                move_group.setPoseTarget(pose_hover);

                if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                    clear_error();
                    continue;
                }

                // Open gripper
                open_gripper();

                move_group.setStartStateToCurrentState();

                if (move_group.computeCartesianPath(down_waypoints, 0.01, trajectory, false) == -1.0) {
                    clear_error();
                    continue;
                }

                if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                    clear_error();
                    continue;
                }

                // Close gripper
                close_gripper();

                // Open gripper
                open_gripper();

                move_group.setStartStateToCurrentState();

                if (move_group.computeCartesianPath(up_waypoints, 0.01, trajectory, false) == -1.0) {
                    clear_error();
                    continue;
                }

                if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                    clear_error();
                    continue;
                }

                success = true;
            }
        }
    }

    //
    // Main Loop
    //

    while (rclcpp::ok()) {
        int pattern = rand() % pattern_poses.size();

        for (int j = 0; j < planning_groups; j++) {
            std::vector<int> block_ids;
            for (unsigned int i = 0; i < pattern_poses[pattern].size(); i++) {
                block_ids.push_back(i);
            }

            if (pattern_random[pattern]) {
                std::random_shuffle(block_ids.begin(), block_ids.end());
            }

            for (int i = 0; i < pattern_poses[pattern].size(); i++) {
                geometry_msgs::msg::Pose pick_pose_hover = block_poses[block_ids[i]];
                pick_pose_hover.position.y += offsets[j];
                pick_pose_hover.position.z = 0.1;

                geometry_msgs::msg::Pose pick_pose = block_poses[block_ids[i]];
                pick_pose.position.y += offsets[j];

                geometry_msgs::msg::Pose place_pose_hover;
                place_pose_hover = pattern_poses[pattern][block_ids[i]];
                place_pose_hover.position.y += offsets[j];
                place_pose_hover.position.z = 0.1;

                geometry_msgs::msg::Pose place_pose;
                place_pose = pattern_poses[pattern][block_ids[i]];
                place_pose.position.y += offsets[j];

                if (gripper_twist) {
                    pick_pose_hover.orientation.x = 0.07;
                    pick_pose_hover.orientation.y = 0.07;
                    pick_pose.orientation.x = 0.07;
                    pick_pose.orientation.y = 0.07;
                    place_pose_hover.orientation.x = 0.07;
                    place_pose_hover.orientation.y = 0.07;
                    place_pose.orientation.x = 0.07;
                    place_pose.orientation.y = 0.07;
                }

                std::vector<geometry_msgs::msg::Pose> pick_down_waypoints;
                pick_down_waypoints.push_back(pick_pose_hover);
                pick_down_waypoints.push_back(pick_pose);

                std::vector<geometry_msgs::msg::Pose> pick_up_waypoints;
                pick_up_waypoints.push_back(pick_pose);
                pick_up_waypoints.push_back(pick_pose_hover);

                std::vector<geometry_msgs::msg::Pose> place_down_waypoints;
                place_down_waypoints.push_back(place_pose_hover);
                place_down_waypoints.push_back(place_pose);

                std::vector<geometry_msgs::msg::Pose> place_up_waypoints;
                place_up_waypoints.push_back(place_pose);
                place_up_waypoints.push_back(place_pose_hover);

                // Pick block
                if (j == 0) {
                    success = false;
                    while (!success) {
                        move_group.setStartStateToCurrentState();
                        move_group.setPoseTarget(pick_pose_hover);

                        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        // Open gripper
                        if (mode == "real") {
                            open_gripper();
                        }

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(pick_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        // Close gripper
                        if (mode == "real" && j == 0) {
                            close_gripper();
                        }

                        // Attach block
                        std::vector<std::string> touch_links = {move_group.getEndEffectorLink(), "table_" + std::to_string(j)};
                        move_group.attachObject("block" + std::to_string(block_ids[i]) + "_" + std::to_string(j), move_group.getEndEffectorLink(), touch_links);

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(pick_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            clear_error();
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            clear_error();
                            break;
                        }

                        success = true;
                    }

                    // Place block
                    success = false;
                    while (!success) {
                        move_group.setStartStateToCurrentState();
                        move_group.setPoseTarget(place_pose_hover);

                        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(place_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        // Open gripper
                        if (mode == "real") {
                            open_gripper();
                        }

                        // Detach block
                        move_group.detachObject(move_group.getEndEffectorLink());

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(place_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            break;
                        }

                        success = true;
                    }
                } else {
                    success = false;
                    while (!success) {
                        move_group_2.setStartStateToCurrentState();
                        move_group_2.setPoseTarget(pick_pose_hover);

                        if (move_group_2.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        move_group_2.setStartStateToCurrentState();

                        if (move_group_2.computeCartesianPath(pick_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        // Attach block
                        std::vector<std::string> touch_links = {move_group_2.getEndEffectorLink(), "table_" + std::to_string(j)};
                        move_group_2.attachObject(prefix + "block" + std::to_string(block_ids[i]) + "_" + std::to_string(j), move_group_2.getEndEffectorLink(), touch_links);

                        move_group_2.setStartStateToCurrentState();

                        if (move_group_2.computeCartesianPath(pick_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            break;
                        }

                        success = true;
                    }

                    // Place block
                    success = false;
                    while (!success) {
                        move_group_2.setStartStateToCurrentState();
                        move_group_2.setPoseTarget(place_pose_hover);

                        if (move_group_2.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        move_group_2.setStartStateToCurrentState();

                        if (move_group_2.computeCartesianPath(place_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        // Detach block
                        move_group_2.detachObject(move_group_2.getEndEffectorLink());

                        move_group_2.setStartStateToCurrentState();

                        if (move_group_2.computeCartesianPath(place_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            break;
                        }

                        success = true;
                    }
                }
            }

            if (j == 0) {
                success = false;
                while (!success) {
                    move_group.setStartStateToCurrentState();
                    move_group.setNamedTarget("home");
                    
                    if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                        if (mode == "real") {
                            clear_error();
                        }
                        continue;
                    }

                    success = true;
                }

                // Clapping
                if (mode == "real") {
                    close_gripper();
                    open_gripper();
                    close_gripper();
                    open_gripper();
                }
            } else {
                success = false;
                while (!success) {
                    move_group_2.setStartStateToCurrentState();
                    move_group_2.setNamedTarget("home");

                    if (move_group_2.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                        continue;
                    }

                    success = true;
                }
            }
        }

        for (int j = 0; j < planning_groups; j++) {
            std::vector<int> block_ids;

            for (unsigned int i = 0; i < pattern_poses[pattern].size(); i++) {
                block_ids.push_back(i);
            }

            if (pattern_random[pattern]) {
                std::random_shuffle(block_ids.begin(), block_ids.end());
            } else {
                std::reverse(block_ids.begin(), block_ids.end());
            }

            for (int i = 0; i < pattern_poses[pattern].size(); i++) {
                geometry_msgs::msg::Pose place_pose_hover = block_poses[block_ids[i]];
                place_pose_hover.position.y += offsets[j];
                place_pose_hover.position.z = 0.1;

                geometry_msgs::msg::Pose place_pose = block_poses[block_ids[i]];
                place_pose.position.y += offsets[j];

                geometry_msgs::msg::Pose pick_pose_hover;
                pick_pose_hover = pattern_poses[pattern][block_ids[i]];
                pick_pose_hover.position.y += offsets[j];
                pick_pose_hover.position.z = 0.1;

                geometry_msgs::msg::Pose pick_pose;
                pick_pose = pattern_poses[pattern][block_ids[i]];
                pick_pose.position.y += offsets[j];

                if (gripper_twist) {
                    pick_pose_hover.orientation.x = 0.07;
                    pick_pose_hover.orientation.y = 0.07;
                    pick_pose.orientation.x = 0.07;
                    pick_pose.orientation.y = 0.07;
                    place_pose_hover.orientation.x = 0.07;
                    place_pose_hover.orientation.y = 0.07;
                    place_pose.orientation.x = 0.07;
                    place_pose.orientation.y = 0.07;
                }

                std::vector<geometry_msgs::msg::Pose> pick_down_waypoints;
                pick_down_waypoints.push_back(pick_pose_hover);
                pick_down_waypoints.push_back(pick_pose);

                std::vector<geometry_msgs::msg::Pose> pick_up_waypoints;
                pick_up_waypoints.push_back(pick_pose);
                pick_up_waypoints.push_back(pick_pose_hover);

                std::vector<geometry_msgs::msg::Pose> place_down_waypoints;
                place_down_waypoints.push_back(place_pose_hover);
                place_down_waypoints.push_back(place_pose);

                std::vector<geometry_msgs::msg::Pose> place_up_waypoints;
                place_up_waypoints.push_back(place_pose);
                place_up_waypoints.push_back(place_pose_hover);

                // Pick block
                if (j == 0) {
                    success = false;
                    while (!success) {
                        move_group.setStartStateToCurrentState();
                        move_group.setPoseTarget(pick_pose_hover);

                        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        // Open gripper
                        if (mode == "real") {
                            open_gripper();
                        }

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(pick_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        // Close gripper
                        if (mode == "real") {
                            close_gripper();
                        }

                        // Attach block
                        std::vector<std::string> touch_links = {move_group.getEndEffectorLink(), "table_" + std::to_string(j)};
                        move_group.attachObject("block" + std::to_string(block_ids[i]) + "_" + std::to_string(j), move_group.getEndEffectorLink(), touch_links);

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(pick_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            break;
                        }

                        success = true;
                    }

                    // Place block
                    success = false;
                    while (!success) {
                        move_group.setStartStateToCurrentState();
                        move_group.setPoseTarget(place_pose_hover);

                        if (move_group.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(place_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        // Open gripper
                        if (mode == "real" && j == 0) {
                            open_gripper();
                        }

                        // Detach block
                        move_group.detachObject(move_group.getEndEffectorLink());

                        move_group.setStartStateToCurrentState();

                        if (move_group.computeCartesianPath(place_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            if (mode == "real") {
                                clear_error();
                            }
                            continue;
                        }

                        if (move_group.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            if (mode == "real") {
                                clear_error();
                            }
                            break;
                        }

                        success = true;
                    }
                } else {
                    success = false;
                    while (!success) {
                        move_group_2.setStartStateToCurrentState();
                        move_group_2.setPoseTarget(pick_pose_hover);

                        if (move_group_2.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        move_group_2.setStartStateToCurrentState();

                        if (move_group_2.computeCartesianPath(pick_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        // Attach block
                        std::vector<std::string> touch_links = {move_group_2.getEndEffectorLink(), "table_" + std::to_string(j)};
                        move_group_2.attachObject("block" + std::to_string(block_ids[i]) + "_" + std::to_string(j), move_group_2.getEndEffectorLink(), touch_links);

                        move_group_2.setStartStateToCurrentState();
                        if (move_group_2.computeCartesianPath(pick_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            break;
                        }

                        success = true;
                    }

                    // Place block
                    success = false;
                    while (!success) {
                        move_group_2.setStartStateToCurrentState();
                        move_group_2.setPoseTarget(place_pose_hover);

                        if (move_group_2.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        move_group_2.setStartStateToCurrentState();

                        if (move_group_2.computeCartesianPath(place_down_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            continue;
                        }

                        // Detach block
                        move_group_2.detachObject(move_group_2.getEndEffectorLink());

                        move_group_2.setStartStateToCurrentState();

                        if (move_group_2.computeCartesianPath(place_up_waypoints, 0.01, trajectory, false) == -1.0) {
                            continue;
                        }

                        if (move_group_2.execute(trajectory) != moveit::core::MoveItErrorCode::SUCCESS) {
                            break;
                        }

                        success = true;
                    }
                }
            }
        }

        gripper_twist = !gripper_twist;
    }

    RCLCPP_INFO(node->get_logger(), "xarm_block_demo finished");
    return 0;
}

// Copyright (C) 2025 Altera Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <chrono>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/logging.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/object_color.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <std_msgs/msg/color_rgba.hpp>
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;

static const std::string NAME = "moveit_demo_client";

std::shared_ptr<rclcpp::Node> node;
geometry_msgs::msg::Pose target_pose;
bool pose_available = false;


void pose_callback(const geometry_msgs::msg::Pose & msg)
{
  target_pose = msg;
  pose_available = true;
}

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);

  node = rclcpp::Node::make_shared(NAME, "", node_options);

  // Parameters
  rclcpp::Parameter random_param;
  rclcpp::Parameter planning_group_param;

  node->get_parameter("random", random_param);

  if (!node->get_parameter("planning_group", planning_group_param)) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Failed to initialize client due to missing 'planning_group' parameter.");
    return false;
  }

  bool random = random_param.as_bool();
  std::string planning_group = planning_group_param.as_string();

  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr planning_publisher = node->create_publisher<std_msgs::msg::Int32>("/" + NAME + "/planning_time", 10);

  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_subscriber = node->create_subscription<geometry_msgs::msg::Pose>(
    "/" + NAME + "/pose", 1, pose_callback);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread([&executor]() { executor.spin(); }).detach();

  moveit::planning_interface::MoveGroupInterface move_group(node, planning_group);
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  move_group.setMaxVelocityScalingFactor(0.3);
  move_group.allowReplanning(true);
  move_group.setReplanAttempts(5);
  move_group.setReplanDelay(1);
  move_group.setPlanningTime(5);

  // Setup scene
  std_msgs::msg::ColorRGBA object_color;
  object_color.r = 1.0;
  object_color.g = 1.0;
  object_color.b = 1.0;
  object_color.a = 0.99;

  std::vector<moveit_msgs::msg::CollisionObject> collision_objects;
  std::vector<moveit_msgs::msg::ObjectColor> object_colors;

  moveit_msgs::msg::CollisionObject table_object;
  table_object.header.frame_id = move_group.getPlanningFrame();
  table_object.id = "table";
  shape_msgs::msg::SolidPrimitive table_primitive;
  table_primitive.type = table_primitive.BOX;
  table_primitive.dimensions.resize(3);
  table_primitive.dimensions[table_primitive.BOX_X] = 2.0;
  table_primitive.dimensions[table_primitive.BOX_Y] = 2.0;
  table_primitive.dimensions[table_primitive.BOX_Z] = 0.1;
  geometry_msgs::msg::Pose table_pose;
  table_pose.position.x = 0.0;
  table_pose.position.y = 0.0;
  table_pose.position.z = -0.05;
  table_object.primitives.push_back(table_primitive);
  table_object.primitive_poses.push_back(table_pose);
  table_object.operation = table_object.ADD;

  moveit_msgs::msg::ObjectColor table_color;
  table_color.id = "table";
  table_color.color = object_color;

  collision_objects.push_back(table_object);
  object_colors.push_back(table_color);

  planning_scene_interface.addCollisionObjects(collision_objects, object_colors);

  while (rclcpp::ok()) {
    move_group.setStartStateToCurrentState();

    if (random) {
      move_group.setRandomTarget();
    } else {
      if (pose_available) {
        move_group.setApproximateJointValueTarget(target_pose);
        pose_available = false;
      } else {
        continue;
      }
    }

    // Plan path
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    auto start_time = std::chrono::high_resolution_clock::now();
    bool success = (move_group.plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();

    if (!success) {
      continue;
    }

    // Trajectory execution
    move_group.execute(plan);

    // Publish planning time
    auto planning_time_msg = std_msgs::msg::Int32();
    planning_time_msg.data = duration;
    planning_publisher->publish(planning_time_msg);
  }

  rclcpp::shutdown();
  return 0;
}

# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.22)
project(moveit_demo_client)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  ament_cmake
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  pluginlib
  rclcpp
)

foreach(Dependency IN ITEMS ${THIS_PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_executable(moveit_demo_client src/moveit_demo_client.cpp)
target_include_directories(moveit_demo_client PUBLIC include)
ament_target_dependencies(moveit_demo_client  ${THIS_PACKAGE_INCLUDE_DEPENDS})

install(TARGETS moveit_demo_client
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()

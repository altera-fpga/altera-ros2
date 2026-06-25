# Copyright (C) 2026 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.16)
project(altera_camera)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

add_library(altera_camera
  src/altera_camera.cpp
  src/altera_camera_device.cpp
)

rclcpp_components_register_nodes(altera_camera "altera_camera::AlteraCamera")

target_include_directories(altera_camera PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(altera_camera
  "rclcpp"
  "rclcpp_components"
  "image_transport"
  "camera_info_manager"
)

target_compile_options(altera_camera PRIVATE -Werror)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(
  altera_camera
  PRIVATE
  "ALTERA_CAMERA_BUILDING_LIBRARY"
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS altera_camera
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

add_executable(altera_camera_node src/altera_camera_node.cpp)

target_include_directories(altera_camera_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(altera_camera_node altera_camera)
target_compile_options(altera_camera_node PRIVATE -Werror)

install(
  TARGETS altera_camera_node
  EXPORT export_${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_targets(
  export_${PROJECT_NAME}
)

ament_export_libraries(
  altera_camera
)

ament_export_dependencies(
  "camera_info_manager"
)

ament_auto_package(
  USE_SCOPED_HEADER_INSTALL_DIR
  INSTALL_TO_SHARE launch
)

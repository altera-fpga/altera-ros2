// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#include "altera_camera/altera_camera.hpp"

#include <algorithm>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

#include <rclcpp/parameter_value.hpp>
#include <rclcpp/qos.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include "altera_camera/altera_camera_device.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using namespace std::chrono_literals;

altera_camera::AlteraCamera::AlteraCamera(rclcpp::NodeOptions const & options)
: rclcpp::Node{"altera_camera", options},
  canceled{false}
{
  // Handle parameters
  publish_fps = declare_parameter("publish_rate", 30);

  auto camera_name = declare_parameter("camera_name", "altera-camera");

  camera_info =
    std::make_shared<camera_info_manager::CameraInfoManager>(
    this, camera_name);

  camera_frame_id = declare_parameter<std::string>(
    "camera_frame_id", camera_name + "_frame");

  auto device_descriptor = rcl_interfaces::msg::ParameterDescriptor{};
  device_descriptor.description = "Path to frame writer device";
  device_descriptor.read_only = true;
  auto device = declare_parameter<std::string>(
    "fw_device", "/dev/uio0", device_descriptor);

  // Setup timer for frame capture
  if (publish_fps > 0) {
    const auto publish_period = rclcpp::Rate(publish_fps).period();
    pub_timer = this->create_wall_timer(
      publish_period,
      [this]() {this->publish_next_frame = true;});
  }

  // Publishers
  camera_pub = image_transport::create_camera_publisher(
    this,
    "image_raw",
    rclcpp::QoS(10).get_rmw_qos_profile());

  // Prepare camera
  camera = std::make_shared<altera_camera::AlteraCameraDevice>(device);

  if (!camera->open()) {
    return;
  }

  // Start capture thread
  capture_thread = std::thread{[this]() -> void {
      // Start capture loop
      while (rclcpp::ok() && !canceled.load()) {
        RCLCPP_DEBUG(get_logger(), "Capture...");

        sensor_msgs::msg::Image::UniquePtr img;
        {
          std::lock_guard<std::mutex> lock(m_lock);
          img = camera->capture();
        }

        if (img == nullptr) {
          continue;
        }

        if (publish_next_frame == false) {
          continue;
        }

        img->header.frame_id = camera_frame_id;

        auto ci = std::make_unique<sensor_msgs::msg::CameraInfo>(
          camera_info->getCameraInfo());

        ci->height = img->height;
        ci->width = img->width;
        ci->header.stamp = img->header.stamp;
        ci->header.frame_id = camera_frame_id;

        camera_pub.publish(*img, *ci);

        publish_next_frame = publish_fps <= 0;
      }
    }
  };
}

altera_camera::AlteraCamera::~AlteraCamera() {
  canceled.store(true);
  if (capture_thread.joinable()) {
    capture_thread.join();
  }

  camera->close();
}

RCLCPP_COMPONENTS_REGISTER_NODE(altera_camera::AlteraCamera)

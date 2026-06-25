// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#ifndef ALTERA_CAMERA__ALTERA_CAMERA_HPP_
#define ALTERA_CAMERA__ALTERA_CAMERA_HPP_

#include "altera_camera/altera_camera_device.hpp"

#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>

#include <ostream>
#include <rclcpp/rclcpp.hpp>
#include <rcl_interfaces/msg/parameter.hpp>

#include <memory>
#include <string>
#include <map>
#include <vector>
#include <optional>
#include <mutex>

namespace altera_camera
{

class AlteraCamera : public rclcpp::Node
{
public:
  explicit AlteraCamera(rclcpp::NodeOptions const & options);

  virtual ~AlteraCamera();

private:
  using ImageSize = std::vector<int64_t>;

  std::shared_ptr<altera_camera::AlteraCameraDevice> camera;

  // Publisher
  image_transport::CameraPublisher camera_pub;

  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info;

  std::thread capture_thread;
  std::atomic<bool> canceled;

  std::string camera_frame_id;

  int publish_fps;
  rclcpp::TimerBase::SharedPtr pub_timer;

  bool publish_next_frame = true;

  std::mutex m_lock;

};

}  // namespace altera_camera

#endif  // ALTERA_CAMERA__ALTERA_CAMERA_HPP_

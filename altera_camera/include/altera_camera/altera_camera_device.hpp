// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#ifndef ALTERA_CAMERA__ALTERA_CAMERA_DEVICE_HPP_
#define ALTERA_CAMERA__ALTERA_CAMERA_DEVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <filesystem>
#include <map>
#include <string>
#include <utility>
#include <tuple>
#include <vector>
#include <optional>

#define FW_VID_PID                     0
#define FW_NUMBER_OF_COLOR_PLANES      28
#define FW_IRQ_CONTROL                 256
#define FW_IRQ_STATUS                  260
#define FW_IMG_WIDTH                   288
#define FW_IMG_HEIGHT                  292
#define FW_STATUS                      320
#define FW_COMMIT                      356
#define FW_BUFFER_ACKNOWLEDGE          360
#define FW_RUN                         364
#define FW_BUFFER_BASE                 372

namespace altera_camera
{

/** 
 * Camera device for Altera Frame Writer over UIO interface.
 */
class AlteraCameraDevice
{
public:
  explicit AlteraCameraDevice(std::string device);

  bool open();
  bool close();

  sensor_msgs::msg::Image::UniquePtr capture();

private:
  struct Buffer
  {
    uint8_t * start;
    int size;
    int offset;
  };

  Buffer ctrl;
  Buffer img;

  std::string device;
  int fd;

};

}  // namespace altera_camera

#endif  // ALTERA_CAMERA__ALTERA_CAMERA_DEVICE_HPP_

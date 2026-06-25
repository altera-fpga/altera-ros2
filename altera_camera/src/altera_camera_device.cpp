// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#include "altera_camera/altera_camera_device.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <fstream>
#include <map>
#include <memory>
#include <regex>
#include <sstream>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>

using altera_camera::AlteraCameraDevice;
using sensor_msgs::msg::Image;

AlteraCameraDevice::AlteraCameraDevice(std::string device)
: device{std::move(device)}
{
}

bool AlteraCameraDevice::open() {
  fd = ::open(device.c_str(), O_RDWR);

  if (fd < 0) {
    auto msg = std::ostringstream{};
    msg << "Failed opening device " << device << ": " << strerror(errno)
        << " (" << errno << ")";
    RCLCPP_ERROR(rclcpp::get_logger("altera_camera"), "%s", msg.str().c_str());
    return false;
  }

  // Get device name
  std::string dev_name = std::filesystem::u8path(device.c_str()).filename();

  const std::filesystem::path sys_path =
    std::filesystem::u8path("/sys/class/uio") /
    std::filesystem::u8path(dev_name);

  const std::filesystem::path c_map =
    sys_path / std::filesystem::u8path("maps/map0");

  const std::filesystem::path i_map =
    sys_path / std::filesystem::u8path("maps/map1");

  if (!std::filesystem::exists(c_map)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("altera_camera"),
        "Unable to find frame writer control map information!");
    return false;
  }

  if (!std::filesystem::exists(i_map)) {
    RCLCPP_ERROR(
        rclcpp::get_logger("altera_camera"),
        "Unable to find frame writer image buffer map information!");
    return false;
  }

  std::filesystem::path c_offset = c_map / std::filesystem::u8path("offset");
  std::filesystem::path c_size = c_map / std::filesystem::u8path("size");

  // Read control offset
  std::string str_c_offset;
  std::ifstream sc_offset(c_offset);

  if (sc_offset.is_open()) {
    sc_offset >> str_c_offset;
    sc_offset.close();
  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("altera_camera"),
        "Failed to read frame writer control map offset!");
    return false;
  }

  // Read size
  std::string str_c_size;
  std::ifstream sc_size(c_size);

  if (sc_size.is_open()) {
    sc_size >> str_c_size;
    sc_size.close();
  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("altera_camera"),
        "Failed to read frame writer control map size!");
    return false;
  }

  // Map buffers
  int page_size = getpagesize();

  ctrl.offset = std::stoul(str_c_offset, nullptr, 16);
  ctrl.size = std::stoul(str_c_size, nullptr, 16);
  void * ctrl_region =
    mmap(0, ctrl.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

  if (ctrl_region == MAP_FAILED) {
    RCLCPP_ERROR(
      rclcpp::get_logger("altera_camera"),
      "Failed to map frame writer control memory!");
    return false;
  }

  ctrl.start = reinterpret_cast<uint8_t *>(ctrl_region);

  std::filesystem::path i_offset = i_map / std::filesystem::u8path("offset");
  std::filesystem::path i_size = i_map / std::filesystem::u8path("size");

  // Read control offset
  std::string str_i_offset;
  std::ifstream si_offset(i_offset);

  if (si_offset.is_open()) {
    si_offset >> str_i_offset;
    si_offset.close();
  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("altera_camera"),
        "Failed to read frame writer image buffer map offset!");
    return false;
  }

  // Read size
  std::string str_i_size;
  std::ifstream si_size(i_size);

  if (si_size.is_open()) {
    si_size >> str_i_size;
    si_size.close();
  } else {
    RCLCPP_ERROR(
        rclcpp::get_logger("altera_camera"),
        "Failed to read frame writer image buffer map size!");
    return false;
  }

  // Check VID_PID
  uint32_t vid_pid;
  std::memcpy(
    &vid_pid,
    ctrl.start + ctrl.offset + FW_VID_PID,
    sizeof(vid_pid));

  if (vid_pid != 0x6AF70249) {
    RCLCPP_DEBUG(
      rclcpp::get_logger("altera_camera"),
      "Device specified is not a valid Altera camera frame writer.");
    return false;
  }

  img.offset = std::stoul(str_i_offset, nullptr, 16);
  img.size = std::stoul(str_i_size, nullptr, 16);
  void * img_region =
    mmap(0, img.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, page_size);

  if (img_region == MAP_FAILED) {
    RCLCPP_ERROR(
      rclcpp::get_logger("altera_camera"),
      "Failed to map frame writer image buffer memory!");
    return false;
  }

  img.start = reinterpret_cast<uint8_t *>(img_region);

  // Set frame writer to single-shot mode
  ctrl.start[ctrl.offset + FW_RUN] = 0x3;
  ctrl.start[ctrl.offset + FW_COMMIT] = 0x1;

  return true;
}

bool AlteraCameraDevice::close() {
  RCLCPP_INFO(rclcpp::get_logger("altera_camera"), "Stopping camera");

  munmap(ctrl.start, ctrl.size);
  munmap(img.start, img.size);
  ::close(fd);

  return true;
}

Image::UniquePtr AlteraCameraDevice::capture() {
  rclcpp::Time timestamp;
  timestamp = rclcpp::Clock{RCL_SYSTEM_TIME}.now();

  // Get image metadata
  uint16_t height;
  uint16_t width;
  uint32_t color_planes;

  std::memcpy(
    &height,
    ctrl.start + ctrl.offset + FW_IMG_HEIGHT,
    sizeof(height));

  std::memcpy(
    &width,
    ctrl.start + ctrl.offset + FW_IMG_WIDTH,
    sizeof(width));

  std::memcpy(
    &color_planes,
    ctrl.start + ctrl.offset + FW_NUMBER_OF_COLOR_PLANES,
    sizeof(color_planes));

  std::string encoding;

  if (color_planes == 1) {
    encoding = sensor_msgs::image_encodings::MONO8;
  } else if (color_planes == 3) {
    encoding = sensor_msgs::image_encodings::BGR8;
  } else {
    RCLCPP_ERROR(
      rclcpp::get_logger("altera_camera"),
      "Unsupported number of color planes reported by frame writer: %u",
      color_planes);
    return nullptr;
  }

  int payload_size = width * height * color_planes;

  // Create image object
  auto image = std::make_unique<Image>();

  // Request frame
  ctrl.start[ctrl.offset + FW_BUFFER_ACKNOWLEDGE] = 0x1;

  // Wait for frame writer to report that the frame is ready.
  volatile uint8_t * status_reg = ctrl.start + ctrl.offset + FW_STATUS;
  while (*status_reg != 0x0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }

  // Copy over buffer data
  image->data.resize(payload_size);
  std::copy_n(img.start + img.offset, payload_size, image->data.begin());

  // Fill in remaining image information
  image->header.stamp = timestamp;
  image->width = width;
  image->height = height;
  image->step = width * color_planes;
  image->encoding = encoding;

  return image;
}



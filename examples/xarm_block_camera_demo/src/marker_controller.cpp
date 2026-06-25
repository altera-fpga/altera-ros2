// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#include "marker_controller.hpp"

#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/mman.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>



MarkerController::MarkerController() {}

bool MarkerController::open() {
  const std::filesystem::path sys_path = std::filesystem::u8path("/sys/class/uio");

  if(!std::filesystem::exists(sys_path) || std::filesystem::is_empty(sys_path)) {
    RCLCPP_FATAL(rclcpp::get_logger("marker_controller"),
                   "No UIO devices detected.");
      return false;
  }

  // Find marker controller device
  for (const auto & entry : std::filesystem::directory_iterator(sys_path)) {
    std::string device = entry.path().filename();
    std::filesystem::path p_name = entry / std::filesystem::u8path("name");

    if (!std::filesystem::exists(p_name)) {
      continue;
    }

    // Read device name
    std::string name;
    std::ifstream s_name(p_name);

    if (s_name.is_open()) {
      s_name >> name;
      s_name.close();

      if (name.find("marker_ctrl") == std::string::npos) {
        continue;
      }
    } else {
      continue;
    }

    // Get memory maps
    std::filesystem::path p_map = entry / std::filesystem::u8path("maps") / std::filesystem::u8path("map0");

    if (!std::filesystem::exists(p_map)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Unable to find marker controller device map information!");
      return false;
    }

    std::filesystem::path p_offset = p_map / std::filesystem::u8path("offset");
    std::filesystem::path p_size = p_map / std::filesystem::u8path("size");

    if (!std::filesystem::exists(p_offset) || !std::filesystem::exists(p_size)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Unable to find marker controller device map offset or size information!");
      return false;
    }

    // Read offset
    std::string str_offset;
    std::ifstream s_offset(p_offset);

    if (s_offset.is_open()) {
      s_offset >> str_offset;
      s_offset.close();
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Unable to read device offset information!");
      return false;
    }

    // Read size
    std::string str_size;
    std::ifstream s_size(p_size);

    if (s_size.is_open()) {
      s_size >> str_size;
      s_size.close();
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Unable to read device size information!");
      return false;
    }

    // Check device path exists
    std::filesystem::path p_dev = std::filesystem::u8path("/dev") / std::filesystem::u8path(device);

    RCLCPP_INFO(rclcpp::get_logger("marker_controller"),
        "Device found: '%s'.", p_dev.c_str());

    if (!std::filesystem::exists(p_dev)) {
      RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Device path '%s' does not exist!", p_dev.c_str());
      return false;
    }

    // Open device
    fd = ::open(p_dev.c_str(), O_RDWR);

    if (fd < 0) {
      auto msg = std::ostringstream{};
      msg << "Failed opening device " << device << ": " << strerror(errno)
          << " (" << errno << ")";
      RCLCPP_ERROR(rclcpp::get_logger("marker_controller"), "%s", msg.str().c_str());
      return false;
    }

    // Map buffers
    int page_size = getpagesize();

    ctrl.offset = std::stoul(str_offset, nullptr, 16);
    ctrl.size = std::stoul(str_size, nullptr, 16);
    void * ctrl_region =
      mmap(0, ctrl.size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);

    if (ctrl_region == MAP_FAILED) {
      RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Failed to map marker controller device memory!");
      return false;
    }

    ctrl.start = reinterpret_cast<uint8_t *>(ctrl_region);

    return true;
  }

  return false;
}

bool MarkerController::close() {
  RCLCPP_INFO(rclcpp::get_logger("marker_controller"), "Stopping marker controller");

  munmap(ctrl.start, ctrl.size);
  ::close(fd);

  return true;
}

void MarkerController::remix() {
  ctrl.start[ctrl.offset + FW_MM] = 0x1;
}

void MarkerController::hide_marker(int id) {
  volatile uint8_t * num_markers = ctrl.start + ctrl.offset + FW_ID;

  if (*num_markers <= id) {
    RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Failed to hide marker with ID %d!", id);

    return;
  }

  uint8_t x = ctrl.start[ctrl.offset + FW_SS];
  ctrl.start[ctrl.offset + FW_SS] = x & ~(1 << id);
}

void MarkerController::show_marker(int id) {
  volatile uint8_t * num_markers = ctrl.start + ctrl.offset + FW_ID;

  if (*num_markers <= id) {
    RCLCPP_ERROR(
        rclcpp::get_logger("marker_controller"),
        "Failed to show marker with ID %d!", id);

    return;
  }

  volatile uint8_t x = ctrl.start[ctrl.offset + FW_SS];
  ctrl.start[ctrl.offset + FW_SS] = x | (1 << id);
}

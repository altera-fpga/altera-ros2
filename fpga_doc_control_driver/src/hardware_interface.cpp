// Copyright (C) 2025 Altera Corporation
// SPDX-License-Identifier: Apache-2.0
//

#include <algorithm>
#include <memory>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <string>
#include <utility>
#include <vector>
#include <math.h>

#include "rclcpp/rclcpp.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "include/hardware_interface.hpp"

namespace fpga_doc_control_driver
{
CallbackReturn DoCSystem::on_init(const hardware_interface::HardwareInfo& info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::filesystem::path sys_path = std::filesystem::u8path("/sys/class/uio");

  if(!std::filesystem::exists(sys_path) || std::filesystem::is_empty(sys_path)) {
      RCLCPP_FATAL(rclcpp::get_logger("DoCSystem"),
                   "No UIO devices detected.");
      return hardware_interface::CallbackReturn::ERROR;
  }

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

      if (name.find("doc") == std::string::npos) {
        continue;
      }
    } else {
      continue;
    }

    // Get axis maps
    std::filesystem::path p_maps = entry / std::filesystem::u8path("maps");

    if (!std::filesystem::exists(p_maps)) {
      continue;
    }

    for (const auto & map : std::filesystem::directory_iterator(p_maps)) {
      std::filesystem::path p_offset = map / std::filesystem::u8path("offset");
      std::filesystem::path p_size = map / std::filesystem::u8path("size");

      if (!std::filesystem::exists(p_offset) || !std::filesystem::exists(p_size)) {
        continue;
      }

      // Read offset
      std::string str_offset;
      std::ifstream s_offset(p_offset);

      if (s_offset.is_open()) {
        s_offset >> str_offset;
        s_offset.close();

      } else {
        continue;
      }

      // Read size
      std::string str_size;
      std::ifstream s_size(p_size);

      if (s_size.is_open()) {
        s_size >> str_size;
        s_size.close();

      } else {
        continue;
      }

      int offset = std::stoul(str_offset, nullptr, 16);
      int size = std::stoul(str_size, nullptr, 16);

      // Check device path exists
      std::filesystem::path p_dev = std::filesystem::u8path("/dev") / std::filesystem::u8path(device);

      RCLCPP_INFO(rclcpp::get_logger("DoCSystem"),
          "DoC device found: '%s'.", p_dev.c_str());

      if (!std::filesystem::exists(p_dev)) {
        continue;
      }

      devices_.emplace_back(p_dev, offset, size);
    }
  }

  // Check joints and device numbers
  num_joints = info.joints.size();

  if (num_joints > devices_.size()) {
      RCLCPP_FATAL(rclcpp::get_logger("DoCSystem"),
                   "%zu joints found but only %zu devices are available.",
                   info.joints.size(),
                   devices_.size());
      return hardware_interface::CallbackReturn::ERROR;
  }

  // Initialize interfaces
  joint_positions_.assign(num_joints, 0.0);
  joint_raw_positions_.assign(num_joints, 0.0);
  joint_velocities_.assign(num_joints, 0.0);
  joint_positions_command_.assign(num_joints, 0.0);
  joint_velocities_command_.assign(num_joints, 0.0);

  int i = 0;
  for (const hardware_interface::ComponentInfo & joint : info.joints) {
    joint_names_.push_back(joint.name);

    for (const auto & interface : joint.command_interfaces) {
      if (interface.name != hardware_interface::HW_IF_POSITION
      && interface.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(rclcpp::get_logger("DoCSystem"),
                   "Joint '%s' has command interface '%s' which is not supported.", joint.name.c_str(),
                   interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    for (const auto & interface : joint.state_interfaces) {
      if (interface.name != hardware_interface::HW_IF_POSITION
      && interface.name != hardware_interface::HW_IF_VELOCITY) {
        RCLCPP_FATAL(rclcpp::get_logger("DoCSystem"),
                   "Joint '%s' has state interface '%s' which is not supported.", joint.name.c_str(),
                   interface.name.c_str());
        return hardware_interface::CallbackReturn::ERROR;
      }

      // Set initial position
      if (interface.name == hardware_interface::HW_IF_POSITION) {
        double pos = std::strtod(interface.initial_value.c_str(), NULL);
        joint_positions_[i] = pos;
      }
    }

    i++;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> DoCSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_names_) {
    state_interfaces.emplace_back(joint_name, "position", &joint_positions_[ind]);
    state_interfaces.emplace_back(joint_name, "raw_position", &joint_raw_positions_[ind]);
    state_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_[ind]);

    ind++;
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> DoCSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  int ind = 0;
  for (const auto & joint_name : joint_names_) {
    command_interfaces.emplace_back(joint_name, "position", &joint_positions_command_[ind]);
    command_interfaces.emplace_back(joint_name, "velocity", &joint_velocities_command_[ind]);

    ind++;
  }

  return command_interfaces;
}

CallbackReturn DoCSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DoCSystem"), "Initializing DoC devices...");

  // Initialize DoC devices
  int i = 0;
  for (DoCDevice& d : devices_) {
    int result = d.activate();
    if (result != 0) {
      RCLCPP_FATAL(rclcpp::get_logger("DoCSystem"),
                  "Failed to initialize device with error code %d", result);
      continue;
    }

    // Set initial position
    d.write(DOC_DBG_POS_SETP0, (int32_t)round((joint_positions_[i] * 32760.0) / M_PI));

    i++;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

CallbackReturn DoCSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("DoCSystem"), "Closing DoC devices...");

  for (DoCDevice& d : devices_) {
    d.deactivate();
  }

  return CallbackReturn::SUCCESS;
}

return_type DoCSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto i = 0ul; i < num_joints; i++) {
    joint_positions_[i] = ((devices_[i].read(DOC_DBG_POSITION) * M_PI) / 32760.0);
    joint_velocities_[i] = ((devices_[i].read(DOC_DBG_SPEED) / 60) * (2 * M_PI));

    // RCLCPP_INFO(rclcpp::get_logger("DoCSystem"), "Position: %g, Speed: %g", joint_positions_[i], joint_velocities_[i]);
  }

  return return_type::OK;
}

return_type DoCSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto i = 0ul; i < num_joints; i++) {
    devices_[i].write(DOC_DBG_POS_SETP0, (int32_t)round((joint_positions_command_[i] * 32760.0) / M_PI));
    joint_raw_positions_[i] = joint_positions_command_[i];
  }

  return return_type::OK;
}

}  // namespace fpga_doc_control_driver

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fpga_doc_control_driver::DoCSystem, hardware_interface::SystemInterface)

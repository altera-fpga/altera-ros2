// Copyright (C) 2025 Altera Corporation
// SPDX-License-Identifier: Apache-2.0
//

#ifndef FPGA_DOC_CONTROL_DRIVER__HARDWARE_INTERFACE_HPP_
#define FPGA_DOC_CONTROL_DRIVER__HARDWARE_INTERFACE_HPP_

// System
#include <memory>
#include <string>
#include <vector>
#include <limits>

// ros2_control hardware_interface
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

// ROS
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

// DoC
#include "doc_device.hpp"

using hardware_interface::return_type;

namespace fpga_doc_control_driver
{
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class DoCSystem : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

protected:
  std::vector<DoCDevice> devices_;

  std::vector<double> joint_positions_command_;
  std::vector<double> joint_positions_;
  std::vector<double> joint_raw_positions_;
  std::vector<double> joint_velocities_command_;
  std::vector<double> joint_velocities_;
  std::vector<std::string> joint_names_;
  double num_joints;
};
}  // namespace fpga_doc_control_driver

#endif  // FPGA_DOC_CONTROL_DRIVER__HARDWARE_INTERFACE_HPP_

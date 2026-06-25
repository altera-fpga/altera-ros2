// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#include "altera_camera/altera_camera.hpp"

#include <memory>

int main(int argc, char ** argv) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<altera_camera::AlteraCamera>(
    rclcpp::NodeOptions{});

  rclcpp::spin(node);
  rclcpp::shutdown();
  node = nullptr;

  return 0;
}

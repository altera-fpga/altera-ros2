// Copyright (C) 2026 Altera Corporation
// SPDX-License-Identifier: Apache-2.0

#include <rclcpp/rclcpp.hpp>

#include <cstdint>
#include <string>

#define FW_RESERVED       3
#define FW_SS             2
#define FW_ID             1
#define FW_MM             0

/** 
 * Controller for marker generator in FPGA.
 */
class MarkerController
{
public:
  explicit MarkerController();

  bool open();
  bool close();

  void remix();
  void hide_marker(int id);
  void show_marker(int id);

private:
  struct Buffer
  {
    uint8_t * start;
    int size;
    int offset;
  };

  Buffer ctrl;

  std::string device;
  int fd;

};

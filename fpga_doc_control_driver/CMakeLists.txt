# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0
#

cmake_minimum_required(VERSION 3.16)
project(fpga_doc_control_driver LANGUAGES CXX)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra)
endif()

set(INCLUDE_DEPENDS
  pluginlib
  rcpputils
  hardware_interface
)

find_package(ament_cmake REQUIRED)
foreach(Dependency IN ITEMS ${INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

add_library(
  fpga_doc_control_driver
  SHARED
  src/doc_device.cpp
  src/hardware_interface.cpp
)
target_include_directories(
  fpga_doc_control_driver
  PRIVATE
  include
)

ament_target_dependencies(
  fpga_doc_control_driver PUBLIC
  ${INCLUDE_DEPENDS}
)

# Causes the visibility macros to use dllexport rather than dllimport,
# which is appropriate when building the dll but not consuming it.
target_compile_definitions(${PROJECT_NAME} PRIVATE "FPGA_DOC_CONTROL_DRIVER_BUILDING_DLL")

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface fpga_doc_control_driver.xml)

# INSTALL
install(
  DIRECTORY src/include/
  DESTINATION include/fpga_doc_control_driver
)

install(TARGETS fpga_doc_control_driver
  EXPORT export_fpga_doc_control_driver
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

## EXPORTS
ament_export_targets(export_fpga_doc_control_driver HAS_LIBRARY_TARGET)
ament_export_dependencies(${INCLUDE_DEPENDS})
ament_package()

# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.10.2)
project(yaskawa_mh24_resources_description)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY meshes urdf DESTINATION share/${PROJECT_NAME})

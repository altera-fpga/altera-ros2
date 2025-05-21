# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: BSD-3-Clause

from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_setup_assistant_launch


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("motoman_mh_24", package_name="yaskawa_mh24_moveit_config")
        .planning_pipelines("stomp", ["stomp", "ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )
    return generate_setup_assistant_launch(moveit_config)

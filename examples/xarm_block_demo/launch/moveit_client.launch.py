# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):

    prefix = LaunchConfiguration('prefix', default='')
    planning_group = LaunchConfiguration('planning_group', default='lite6')
    planning_group_2 = LaunchConfiguration('planning_group_2', default='')
    mode = LaunchConfiguration('mode', default='fake')
    offset = LaunchConfiguration('offset', default=0.0)

    moveit_client_node = Node(
        name='xarm_block_demo',
        package='xarm_block_demo',
        executable='xarm_block_demo',
        namespace=prefix,
        parameters=[
            {
                'prefix': prefix,
                'planning_group': planning_group,
                'planning_group_2': planning_group_2,
                'mode': mode,
                'offset': offset
            }
        ],
    )

    return [
        moveit_client_node
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

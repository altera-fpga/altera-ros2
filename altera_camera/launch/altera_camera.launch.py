# Copyright (C) 2026 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def launch_setup(context, *args, **kwargs):

    altera_camera_node = Node(
        package="altera_camera",
        executable="altera_camera_node",
        namespace=LaunchConfiguration("altera_camera_namespace"),
        parameters=[
            {
                "fw_device": LaunchConfiguration("fw_device"),
                "camera_name": LaunchConfiguration("camera_name"),
                "publish_rate": LaunchConfiguration("publish_rate"),
            },
        ],
    )

    return [altera_camera_node]

def generate_launch_description():

    launch_arguments = []

    launch_arguments.append(DeclareLaunchArgument("fw_device", default_value="/dev/uio0",
                   description="Path to frame writer device"))
    launch_arguments.append(DeclareLaunchArgument("altera_camera_namespace", default_value="/",
                   description="Namespace in which to launch node"))
    launch_arguments.append(DeclareLaunchArgument("camera_name", default_value="altera-camera",
                   description="Name of the camera. This is used to find the correct camera information file."))
    launch_arguments.append(DeclareLaunchArgument("publish_rate", default_value="-1",
                   description="Number of frames to attempt to publish per second. Value <= 0 means no limitation on publish rate"))

    return LaunchDescription(
        [
            *launch_arguments,
            OpaqueFunction(function=launch_setup),
        ]
    )

# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: BSD-3-Clause

import os
from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix", default="")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware", default="false")
    use_rviz = LaunchConfiguration("use_rviz", default="true")

    moveit_config = (
        MoveItConfigsBuilder("yaskawa_mh24")
        .robot_description(file_path="config/motoman_mh_24.urdf.xacro", mappings={"use_mock_hardware": use_mock_hardware})
        .planning_pipelines("stomp", ["stomp", "ompl", "chomp", "pilz_industrial_motion_planner"])
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .to_moveit_configs()
    )

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--log-level", "debug"],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name='{}static_transform_publisher'.format(prefix.perform(context)),
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='{}robot_state_publisher'.format(prefix.perform(context)),
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ROS 2 controllers
    ros2_controllers_path = os.path.join(
        get_package_share_directory("yaskawa_mh24_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )

    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            ros2_controllers_path,
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "mh24_arm_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {} -c /controller_manager".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # RViz
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]
 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz.perform(context)),
        output="log",
        respawn=False,
        arguments=["-d", str(moveit_config.package_path / "config/moveit.rviz")],
        parameters=rviz_parameters,
    )

    return [
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        rviz_node,
    ] + load_controllers

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "use_mock_hardware",
            default_value="false",
            description="Start robot with mock hardware.",
        ),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        OpaqueFunction(function=launch_setup)
    ])

# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder


def launch_setup(context, *args, **kwargs):
    prefix = LaunchConfiguration("prefix", default="")
    random = LaunchConfiguration("random", default="true")
    mode = LaunchConfiguration("mode", default="fake")
    robot_ip = LaunchConfiguration("robot_ip", default="0.0.0.0")
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    plugin = "uf_robot_hardware/UFRobotFakeSystemHardware"

    if mode.perform(context) == 'real':
        plugin = "uf_robot_hardware/UFRobotSystemHardware"

    if mode.perform(context) == 'doc':
        plugin = "fpga_doc_control_driver/DoCSystem"

    moveit_config = (
        MoveItConfigsBuilder(
            context=context,
            robot_ip=robot_ip,
            controllers_name="controllers",
            ros2_control_plugin=plugin,
            robot_type='lite',
            dof=6,
            add_gripper=True,
            limited=False,
        )
        .robot_description()
        .trajectory_execution(file_path="config/lite6/controllers.yaml")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .to_moveit_configs()
    )

    config = moveit_config.to_dict()
    config.update({
        "random": random,
        "planning_group": "lite6",
    })

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
        get_package_share_directory("xarm_controller"),
        "config",
        "lite6_controllers.yaml",
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
        "lite6_traj_controller",
        "joint_state_broadcaster",
    ]:
        load_controllers += [
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner {} -c /controller_manager".format(controller)],
                shell=True,
                output="screen",
            )
        ]

    # MoveIt demo client executable
    moveit_demo_client_cpp = Node(
        name="moveit_demo_client",
        package="moveit_demo_client",
        executable="moveit_demo_client",
        output="log",
        parameters=[config],
    )

    # RViz
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    rviz_config_path = os.path.join(
        get_package_share_directory("moveit_demo_client"),
        "config",
        "moveit_demo.rviz",
    )
 
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        condition=IfCondition(use_rviz.perform(context)),
        output="log",
        respawn=False,
        arguments=["-d", rviz_config_path],
        parameters=rviz_parameters,
    )

    return [
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        moveit_demo_client_cpp,
        rviz_node,
    ] + load_controllers

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="fake",
            description="Run demo client with real robot arm ('real'), Drive-on-Chip controller ('doc') or 'fake' controller.",
        ),
        DeclareLaunchArgument(
            "robot_ip",
            default_value="0.0.0.0",
            description="IP address of real robot arm controller (optional)",
        ),
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup)
    ])

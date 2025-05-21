# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory
from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    robot_ip = LaunchConfiguration('robot_ip')
    prefix = LaunchConfiguration('prefix', default="")
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    # Robot Driver
    robot_driver_launch = IncludeLaunchDescription(
         PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_api'), 'launch', '_robot_driver.launch.py'])),
         launch_arguments={
             'robot_ip': robot_ip,
             'dof': '6',
             'add_gripper': 'true',
             'robot_type': 'lite',
         }.items(),
    )

    # Load the robot configuration
    moveit_config = (
        MoveItConfigsBuilder(
            controllers_name="controllers",
            robot_ip=robot_ip,
            ros2_control_plugin="uf_robot_hardware/UFRobotSystemHardware",
            context=context,
            robot_type="lite",
            dof=6,
            add_gripper=True,
            prefix=prefix,
            limited=True,
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
            ros2_controllers_path
        ],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        output="both",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["lite6_traj_controller", "-c", "/controller_manager"],
    )

    moveit_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_block_demo'), 'launch', 'moveit_client.launch.py'])),
        launch_arguments={
            'planning_group': '{}lite6'.format(prefix.perform(context)),
            'mode': 'real'
        }.items(),
    )

    # RViz
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    rviz_config_path = os.path.join(
        get_package_share_directory("xarm_block_demo"),
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
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[
                    moveit_client_launch,
                    rviz_node,
                ]
            )
        ),
        robot_driver_launch,
        static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup)
    ])

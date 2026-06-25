# Copyright (C) 2026 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit

from launch_ros.actions import Node

from uf_ros_lib.moveit_configs_builder import MoveItConfigsBuilder

def launch_setup(context, *args, **kwargs):

    prefix = LaunchConfiguration('prefix', default="")
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    # Load the robot configuration
    mg_builder = MoveItConfigsBuilder(
        controllers_name="controllers",
        ros2_control_plugin="fpga_doc_control_driver/DoCSystem",
        context=context,
        robot_type="lite",
        dof=6,
        add_gripper=True,
        prefix=prefix,
        limited=True,
        add_generic_camera=True,
        add_camera_links=True,
    )

    mg_builder.robot_description()
    mg_builder.trajectory_execution(file_path="config/lite6/controllers.yaml")
    mg_builder.planning_scene_monitor(
        publish_robot_description=True,
        publish_robot_description_semantic=True
    )

    mg_builder.planning_pipelines(pipelines=["ompl"])

    moveit_config = mg_builder.to_moveit_configs()

    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--log-level", "debug"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name=f'{prefix.perform(context)}robot_state_publisher',
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

    moveit_client_parameters = {
        'planning_group': 'lite6',
        'mode': 'fake',
        'use_marker_generator': True,
        'offset_z': -0.8,
        'offset_x': 0.0,
        'offset_y': 0.0,
    }

    moveit_client_node = Node(
        name='xarm_block_camera_demo',
        package='xarm_block_camera_demo',
        executable='xarm_block_camera_demo',
        respawn=True,
        respawn_delay=2,
        parameters=[moveit_client_parameters],
    )

    # RViz
    rviz_parameters = [
        moveit_config.planning_pipelines,
        moveit_config.robot_description_kinematics,
        moveit_config.joint_limits,
    ]

    rviz_config_path = os.path.join(
        get_package_share_directory("xarm_block_camera_demo"),
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

    # Altera Camera
    altera_camera_parameters = {
        "camera_frame_id": "xarm_camera_color_optical_frame",
        "publish_rate": 6,
        "fw_device": "/dev/uio1",
    }

    altera_camera_node = Node(
        package="altera_camera",
        executable="altera_camera_node",
        output="log",
        parameters=[altera_camera_parameters],
        namespace="/camera/camera/color"
    )

    aruco_parameters = {
        'image_topic': '/camera/camera/color/image_raw',
        'camera_info_topic': '/camera/camera/color/camera_info',
        'marker_size': 0.020,
        'camera_frame': 'xarm_camera_color_optical_frame',
        'dictionary': 'DICT_4X4_50',
        'undistort_image': True,
    }

    aruco_node = Node(
        package='aruco_markers',
        executable='aruco_markers',
        parameters=[aruco_parameters],
        output="log",
    )

    launch_list =  [
        RegisterEventHandler(
            OnProcessExit(
                target_action=arm_controller_spawner,
                on_exit=[
                    moveit_client_node,
                    rviz_node,
                ]
            )
        ),
        robot_state_publisher,
        run_move_group_node,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        arm_controller_spawner,
        altera_camera_node,
        aruco_node
    ]

    return launch_list

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup)
    ])

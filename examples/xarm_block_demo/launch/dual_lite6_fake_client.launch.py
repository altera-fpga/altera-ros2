# Copyright (C) 2025 Altera Corporation
# SPDX-License-Identifier: Apache-2.0

import os
import yaml

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
from uf_ros_lib.moveit_configs_builder import DualMoveItConfigsBuilder
from uf_ros_lib.uf_robot_utils import generate_dual_ros2_control_params_temp_file

def launch_setup(context, *args, **kwargs):

    prefix_1 = LaunchConfiguration('prefix_1', default='L_')
    prefix_2 = LaunchConfiguration('prefix_2', default='R_')
    plugin_1 = LaunchConfiguration('plugin_1', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    plugin_2 = LaunchConfiguration('plugin_2', default='uf_robot_hardware/UFRobotFakeSystemHardware')
    use_rviz = LaunchConfiguration("use_rviz", default="false")

    # Load the robot configuration
    moveit_config = (
        DualMoveItConfigsBuilder(
            controllers_name="fake_controllers",
            ros2_control_plugin_1=plugin_1,
            ros2_control_plugin_2=plugin_2,
            context=context,
            robot_type_1="lite",
            robot_type_2="lite",
            dof_1=6,
            dof_2=6,
            add_gripper_1=True,
            add_gripper_2=True,
            prefix_1=prefix_1,
            prefix_2=prefix_2,
            limited=True,
        )
        .robot_description()
        .trajectory_execution(file_path="config/lite6/fake_controllers.yaml")
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
    l_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_1.perform(context)),
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'world', 'L_base_link'],
    )
    r_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='{}static_transform_publisher'.format(prefix_2.perform(context)),
        output='screen',
        arguments=['0.0', '1.0', '0.0', '0.0', '0.0', '0.0', 'world', 'R_base_link'],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name='robot_state_publisher',
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ROS 2 controllers
    ros2_control_params = generate_dual_ros2_control_params_temp_file(
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'lite6_controllers.yaml'),
        os.path.join(get_package_share_directory('xarm_controller'), 'config', 'lite6_controllers.yaml'),
        prefix_1=prefix_1.perform(context),
        prefix_2=prefix_2.perform(context),
        add_gripper_1='True',
        add_gripper_2='True',
        robot_type_1='lite',
        robot_type_2='lite',
    )

    ros2_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_controller'), 'launch', '_dual_ros2_control.launch.py'])),
        launch_arguments={
            'robot_description': yaml.dump(moveit_config.robot_description),
            'ros2_control_params': ros2_control_params,
        }.items(),
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

    l_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["L_lite6_traj_controller", "-c", "/controller_manager"],
    )

    r_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["R_lite6_traj_controller", "-c", "/controller_manager"],
    )

    moveit_client_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([FindPackageShare('xarm_block_demo'), 'launch', 'moveit_client.launch.py'])),
        launch_arguments={
            'planning_group': 'L_lite6',
            'planning_group_2': 'R_lite6',
            'mode': 'fake',
            'offset': '1.0',
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
                target_action=joint_state_broadcaster_spawner,
                on_exit=[
                    moveit_client_launch,
                    rviz_node,
                ]
            )
        ),
        l_static_tf,
        r_static_tf,
        robot_state_publisher,
        run_move_group_node,
        ros2_control_launch,
        joint_state_broadcaster_spawner,
        l_arm_controller_spawner,
        r_arm_controller_spawner,
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_rviz", default_value="false"),
        OpaqueFunction(function=launch_setup)
    ])

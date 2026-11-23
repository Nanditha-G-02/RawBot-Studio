# Copyright 2026 ROBOTIS CO., LTD.
# Licensed under the Apache License, Version 2.0
"""UFACTORY arm in ros2_control mode without RViz or MoveIt, for RawBot Studio: UFACTORY's
_robot_ros2_control launch (hardware interface connected to the controller, robot_state_publisher,
joint_state_publisher: /<hw_ns>/joint_states -> /joint_states) plus the trajectory controller
spawner. The controller starts by holding the current position."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def setup(context):
    rt = LaunchConfiguration('robot_type').perform(context)
    dof = LaunchConfiguration('dof').perform(context)
    name = '{}{}'.format(rt, dof if rt in ('xarm', 'lite') else '')
    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('xarm_controller'), 'launch', '_robot_ros2_control.launch.py'])),
        launch_arguments={
            'robot_ip': LaunchConfiguration('robot_ip'),
            'report_type': 'normal',
            'hw_ns': 'ufactory',
            'robot_type': rt,
            'dof': dof,
            'add_gripper': LaunchConfiguration('add_gripper'),
            'add_vacuum_gripper': LaunchConfiguration('add_vacuum_gripper'),
        }.items(),
    )
    spawn = Node(package='controller_manager', executable='spawner', output='screen',
                 arguments=['{}_traj_controller'.format(name),
                            '--controller-manager', '/controller_manager'])
    return [control, spawn]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_ip'),
        DeclareLaunchArgument('robot_type', default_value='lite'),
        DeclareLaunchArgument('dof', default_value='6'),
        DeclareLaunchArgument('add_gripper', default_value='false'),
        DeclareLaunchArgument('add_vacuum_gripper', default_value='false'),
        OpaqueFunction(function=setup),
    ])

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Online Asynchronous SLAM Launch File
Koristi slam_toolbox za mapiranje okoline u realnom vremenu

Usage:
    ros2 launch student_assignment_02 online_async_launch.py
"""

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Koristi student_assignment_02 paket
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')
    
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_use_sim_time_argument = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation/Gazebo clock')
    
    # Direktna path do slam params - bez LaunchConfiguration
    slam_params_file = os.path.join(
        student_config_dir, 
        'mapper_params_online_async.yaml')

    # Stage koristi 'laser' frame za /base_scan
    # Trebamo transform: laser -> base_link
    laser_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.15', '0', '0', '0', '1', 'laser', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Asynchronous SLAM Toolbox Node
    start_async_slam_toolbox_node = Node(
        parameters=[
          slam_params_file,
          {'use_sim_time': use_sim_time}
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        remappings=[
            ('/scan', '/base_scan'),  # Stage koristi /base_scan
            ('tf', 'tf'),
            ('tf_static', 'tf_static'),
        ]
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_argument)
    ld.add_action(laser_to_base_link_tf)
    ld.add_action(start_async_slam_toolbox_node)

    return ld

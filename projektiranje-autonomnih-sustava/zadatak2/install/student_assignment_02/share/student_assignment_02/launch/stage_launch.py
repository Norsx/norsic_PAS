#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Stage Simulator Launch File - Adapted for student_assignment_02
Koristi world datoteke iz student_assignment_02/world direktorija

Usage:
    ros2 launch student_assignment_02 stage_launch.py
    ros2 launch student_assignment_02 stage_launch.py world:=map_01
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    # Koristi student_assignment_02 paket
    student_share = get_package_share_directory('student_assignment_02')
    student_world_dir = os.path.join(student_share, 'world')

    use_stamped_velocity = LaunchConfiguration('use_stamped_velocity')
    use_stamped_velocity_arg = DeclareLaunchArgument(
        'use_stamped_velocity',
        default_value='false',
        description='on true stage will accept TwistStamped command messages')
    
    enforce_prefixes = LaunchConfiguration('enforce_prefixes')
    enforce_prefixes_arg = DeclareLaunchArgument(
        'enforce_prefixes',
        default_value='false',
        description='on true a prefixes are used for a single robot environment')
    
    use_static_transformations = LaunchConfiguration('use_static_transformations')
    use_static_transformations_arg = DeclareLaunchArgument(
        'use_static_transformations',
        default_value='false',
        description='Use static transformations for sensor frames')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    # Direktna path do world datoteke (bez LaunchConfiguration u os.path.join)
    world_file = os.path.join(student_world_dir, 'map_01.world')
    
    # Stage node
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'world_file': world_file,
            'enforce_prefixes': enforce_prefixes,
            'use_stamped_velocity': use_stamped_velocity,
            'use_static_transformations': use_static_transformations,
            'publish_ground_truth': True,
        }],
        remappings=[
            ('/odom', '/odom'),
            ('/base_pose_ground_truth', '/ground_truth'),
        ]
    )

    return LaunchDescription([
        use_stamped_velocity_arg,
        enforce_prefixes_arg, 
        use_static_transformations_arg,
        use_sim_time_arg,
        stage_node,
    ])

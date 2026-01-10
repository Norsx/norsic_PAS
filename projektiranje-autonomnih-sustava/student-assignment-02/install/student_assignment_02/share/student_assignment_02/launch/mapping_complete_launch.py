#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete Mapping Launch File
Pokreće:
1. RViz s vizualizacijom
2. Stage simulator sa map_01.world
3. SLAM toolbox za mapiranje
4. Potrebne transformacije

Usage:
    ros2 launch student_assignment_02 mapping_complete_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Dobij direktorij našeg paketa
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')
    student_world_dir = os.path.join(student_share, 'world')

    # Argumenti
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    declare_rviz = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')

    # Direktna path do world datoteke
    world_file = os.path.join(student_world_dir, 'map_01.world')
    slam_params_file = os.path.join(student_config_dir, 'mapper_params_online_async.yaml')

    # ====================================================================
    # 1. STAGE SIMULATOR
    # ====================================================================
    stage_node = Node(
        package='stage_ros2',
        executable='stage_ros2',
        name='stage',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'world_file': world_file,
            'enforce_prefixes': False,
            'use_stamped_velocity': False,
            'use_static_transformations': False,  # Dinamičke transformacije!
            'publish_ground_truth': True,
        }],
        remappings=[
            ('/odom', '/odom'),
            ('/base_pose_ground_truth', '/ground_truth'),
        ]
    )

    # ====================================================================
    # 2. STATIC TF: laser -> base_link (Za SLAM mapiranje)
    # ====================================================================
    laser_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '-0.15', '0', '0', '0', '1', 'laser', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # 3. ROBOT STATE PUBLISHER - Za TF između base_link i base_scan
    # ====================================================================
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': """
<robot name="robot">
  <link name="base_link"/>
  <link name="laser"/>
  <joint name="laser_joint" type="fixed">
    <parent link="base_link"/>
    <child link="laser"/>
    <origin xyz="0 0 0.15" rpy="0 0 0"/>
  </joint>
</robot>
            """
        }],
        remappings=[]
    )

    # ====================================================================
    # 4. SLAM TOOLBOX - Online Asynchronous Mapping
    # ====================================================================
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        emulate_tty=True,
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time}
        ],
        remappings=[
            ('/scan', '/base_scan'),  # Stage koristi /base_scan
            ('tf', 'tf'),
            ('tf_static', 'tf_static'),
        ]
    )

    # ====================================================================
    # 5. RVIZ - Vizualizacija
    # ====================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d', os.path.join(student_config_dir, 'rviz_config.rviz')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # Kombiniraj sve
    # ====================================================================
    return LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        stage_node,
        laser_to_base_link_tf,
        robot_state_publisher,
        slam_toolbox_node,
        rviz_node,
    ])

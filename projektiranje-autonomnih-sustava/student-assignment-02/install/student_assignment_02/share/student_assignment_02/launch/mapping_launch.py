#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Mapping Launch File
Pokreće Stage simulator sa SLAM toolbox-om za mapiranje okoline

USE:
    ros2 launch student_assignment_02 mapping_launch.py

ALTERNATIVE - Koristi novi mapping_complete_launch.py koji ima sve!
    ros2 launch student_assignment_02 mapping_complete_launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get package directories
    student_share = get_package_share_directory('student_assignment_02')
    launch_dir = os.path.join(student_share, 'launch')
    
    # Launch arguments
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
    
    # Include Stage launch
    stage_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'stage_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # Include SLAM launch (online asynchronous SLAM)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
        }.items()
    )
    
    # TREBALO BI VIDJETI RViz - trebamo ga dodati!
    # ALI - koristi umjesto toga mapping_complete_launch.py koji ima sve!
    
    ld = LaunchDescription([
        declare_use_sim_time,
        declare_rviz,
        stage_launch,
        slam_launch,
    ])
    
    return ld

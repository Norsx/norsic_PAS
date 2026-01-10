#!/usr/bin/env python3
"""
Navigation Launch File - MINIMALISTIKA VERZIJA
BEZ Lifecycle Managera (koji nije dostupan)

RJEŠENJE:
- A* Path Planner - planira putanju
- Nav2 Adapter - slijedi putanju bez controller servera
- Robot se kreće prema cilju

Evo čini ovakvo:
1. RViz "2D Goal Pose" → /goal_pose
2. A* node planira putanju → /planned_path
3. Nav2 Adapter hvata putanju
4. Nav2 Adapter usmjerava robota prema cilju
5. Robot se kreće

BEZ potrebe za Lifecycle Managerom ili Local Costmapom!
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_dir = get_package_share_directory('student_assignment_02')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # =====================================================================
    # A* PATH PLANNER NODE
    # =====================================================================
    # Planira putanju od base_link do goal_pose
    # Objavljuje je na /planned_path
    astar_planner = Node(
        package='student_assignment_02',
        executable='astar_path_planner_node',
        name='a_star_path_planner',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'allow_diagonal': True},
            {'inflation_distance_m': 0.5},
            {'max_iterations': 50000},
        ]
    )
    
    # =====================================================================
    # NAV2 ADAPTER NODE
    # =====================================================================
    # Hvata putanju od A* planera
    # Usmjerava robota prema cilju
    # Bez potrebe za controller serverom!
    nav2_adapter = Node(
        package='student_assignment_02',
        executable='nav2_adapter_node',
        name='nav2_adapter',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
        ]
    )
    
    # =====================================================================
    # LAUNCH DESCRIPTION
    # =====================================================================
    # Jednostavno: samo dva čvora
    ld = LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Koristi simulacijsko vrijeme'
        ),
        
        # Redoslijed:
        # 1. A* planer - planira putanju
        astar_planner,
        
        # 2. Adapter - slijedi putanju
        nav2_adapter,
    ])
    
    return ld

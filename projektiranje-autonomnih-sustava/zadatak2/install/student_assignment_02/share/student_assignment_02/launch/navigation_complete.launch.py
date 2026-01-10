#!/usr/bin/env python3
"""
Navigation Complete Launch File
Integrira A* path planning s path following (Pure Pursuit)
Za korištenje: ros2 launch student_assignment_02 navigation_complete.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file za A* planiranje putanje i slijeđenje"""
    
    # Paket direktorij
    student_pkg_dir = get_package_share_directory('student_assignment_02')
    
    # =====================================================================
    # LAUNCH ARGUMENTI - A* PLANER
    # =====================================================================
    goal_x_arg = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal X koordinata (m)'
    )
    goal_y_arg = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y koordinata (m)'
    )
    start_x_arg = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Start X koordinata (m)'
    )
    start_y_arg = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Start Y koordinata (m)'
    )
    allow_diagonal_arg = DeclareLaunchArgument(
        'allow_diagonal',
        default_value='true',
        description='Dozvoli dijagonalno kretanje'
    )
    inflation_radius_arg = DeclareLaunchArgument(
        'inflation_radius',
        default_value='1',
        description='Inflation radius oko prepreka (stanice)'
    )
    inflation_distance_arg = DeclareLaunchArgument(
        'inflation_distance_m',
        default_value='0.5',
        description='Inflation buffer distanca (metri)'
    )
    
    # =====================================================================
    # LAUNCH ARGUMENTI - PATH FOLLOWER
    # =====================================================================
    lookahead_distance_arg = DeclareLaunchArgument(
        'lookahead_distance',
        default_value='0.3',
        description='Lookahead distanca za Pure Pursuit (m)'
    )
    max_linear_vel_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='0.5',
        description='Maksimalna linearna brzina (m/s)'
    )
    max_angular_vel_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='1.0',
        description='Maksimalna kutna brzina (rad/s)'
    )
    
    # =====================================================================
    # LAUNCH ARGUMENTI - VIZUALIZACIJA
    # =====================================================================
    rviz_enabled_arg = DeclareLaunchArgument(
        'rviz_enabled',
        default_value='true',
        description='Pokreni RViz za vizualizaciju'
    )
    
    # =====================================================================
    # 1. A* PATH PLANNER NODE
    # =====================================================================
    astar_node = Node(
        package='student_assignment_02',
        executable='a_star_path_planner',
        name='a_star_path_planner',
        output='screen',
        parameters=[
            {
                'goal_x': LaunchConfiguration('goal_x'),
                'goal_y': LaunchConfiguration('goal_y'),
                'start_x': LaunchConfiguration('start_x'),
                'start_y': LaunchConfiguration('start_y'),
                'allow_diagonal': LaunchConfiguration('allow_diagonal'),
                'inflation_radius': LaunchConfiguration('inflation_radius'),
                'inflation_distance_m': LaunchConfiguration('inflation_distance_m'),
                'inflation_cost_threshold': 60,
                'max_iterations': 50000,
                'search_radius': -1,  # -1 = bez ograničenja
            }
        ],
        remappings=[
            ('/map', '/map'),
            ('/goal_pose', '/goal_pose'),
            ('/planned_path', '/planned_path'),
        ]
    )
    
    # =====================================================================
    # 2. PATH FOLLOWER NODE (Pure Pursuit)
    # =====================================================================
    path_follower_node = Node(
        package='student_assignment_02',
        executable='path_follower',
        name='path_follower',
        output='screen',
        parameters=[
            {
                'lookahead_distance': LaunchConfiguration('lookahead_distance'),
                'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
                'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
                'path_topic': '/planned_path',
                'cmd_vel_topic': '/cmd_vel',
                'goal_tolerance': 0.1,
            }
        ],
        remappings=[
            ('planned_path', '/planned_path'),
            ('cmd_vel', '/cmd_vel'),
        ]
    )
    
    # =====================================================================
    # 4. LAUNCH DESCRIPTION
    # =====================================================================
    ld = LaunchDescription([
        # Argumenti
        goal_x_arg,
        goal_y_arg,
        start_x_arg,
        start_y_arg,
        allow_diagonal_arg,
        inflation_radius_arg,
        inflation_distance_arg,
        lookahead_distance_arg,
        max_linear_vel_arg,
        max_angular_vel_arg,
        rviz_enabled_arg,
        
        # Čvorovi
        astar_node,
        path_follower_node,  # Path follower aktiviran!
    ])
    
    return ld

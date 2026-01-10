#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Complete Localization Launch File
Pokreće:
1. RViz s vizualizacijom (koristi rviz_navigation.rviz config)
2. Stage simulator sa map_01.world
3. Localization stack s mapiranom mapom (map_01)
4. Potrebne transformacije

Usage:
    ros2 launch student_assignment_02 localization_complete_launch.py
    
Alternativno s custom mapom:
    ros2 launch student_assignment_02 localization_complete_launch.py map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.descriptions import ComposableNode, ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Dobij direktorij našeg paketa
    student_share = get_package_share_directory('student_assignment_02')
    student_config_dir = os.path.join(student_share, 'config')
    student_world_dir = os.path.join(student_share, 'world')
    student_mapped_maps_dir = os.path.join(student_share, 'mapped_maps')

    # ====================================================================
    # LAUNCH ARGUMENTI
    # ====================================================================
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

    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(student_mapped_maps_dir, 'map_01', 'map_01.yaml'),
        description='Full path to map yaml file to load'
    )

    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(student_config_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_use_composition = DeclareLaunchArgument(
        'use_composition',
        default_value='false',
        description='Use composed bringup if True'
    )

    declare_container_name = DeclareLaunchArgument(
        'container_name',
        default_value='nav2_container',
        description='The name of container that nodes will load in if use composition'
    )

    declare_use_respawn = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Whether to respawn if a node crashes. Applied when composition is disabled.'
    )

    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='log level'
    )

    # Launch Configuration varijable
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    namespace = LaunchConfiguration('namespace')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')

    # Lifecycle nodes
    lifecycle_nodes = ['map_server', 'amcl']

    # Remappings za TF
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]

    # ====================================================================
    # 1. STAGE SIMULATOR
    # ====================================================================
    world_file = os.path.join(student_world_dir, 'map_01.world')
    
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
            'use_static_transformations': False,
            'publish_ground_truth': True,
        }],
        remappings=[
            ('/odom', '/odom'),
            ('/base_pose_ground_truth', '/ground_truth'),
        ]
    )

    # ====================================================================
    # 2. STATIC TF: laser -> base_link (Za lokalizaciju)
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
    # 4. PARAMETRI ZA LOCALIZATION
    # ====================================================================
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': map_yaml_file
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites=param_substitutions,
            convert_types=True),
        allow_substs=True
    )

    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )

    # ====================================================================
    # 5. LOCALIZATION NODES (MAP SERVER + AMCL + LIFECYCLE MANAGER)
    # ====================================================================
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(["'", use_composition, "' == 'false'"])),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='map_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_amcl',
                executable='amcl',
                name='amcl',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_localization',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }])
        ]
    )

    load_composable_nodes = LoadComposableNodes(
        condition=IfCondition(PythonExpression(["'", use_composition, "' == 'true'"])),
        target_container=container_name_full,
        composable_node_descriptions=[
            ComposableNode(
                package='nav2_map_server',
                plugin='nav2_map_server::MapServer',
                name='map_server',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_amcl',
                plugin='nav2_amcl::AmclNode',
                name='amcl',
                parameters=[configured_params],
                remappings=remappings),
            ComposableNode(
                package='nav2_lifecycle_manager',
                plugin='nav2_lifecycle_manager::LifecycleManager',
                name='lifecycle_manager_localization',
                parameters=[{
                    'use_sim_time': use_sim_time,
                    'autostart': autostart,
                    'node_names': lifecycle_nodes
                }]),
        ],
    )

    # ====================================================================
    # 6. RVIZ - Vizualizacija
    # ====================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        condition=IfCondition(use_rviz),
        arguments=[
            '-d', os.path.join(student_config_dir, 'rviz_navigation.rviz')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ====================================================================
    # KOMBINIRAJ SVE
    # ====================================================================
    ld = LaunchDescription()

    # Set environment variables
    ld.add_action(stdout_linebuf_envvar)

    # Declare the launch options
    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_rviz)
    ld.add_action(declare_namespace)
    ld.add_action(declare_map_yaml)
    ld.add_action(declare_params_file)
    ld.add_action(declare_autostart)
    ld.add_action(declare_use_composition)
    ld.add_action(declare_container_name)
    ld.add_action(declare_use_respawn)
    ld.add_action(declare_log_level)

    # Add simulator nodes
    ld.add_action(stage_node)
    ld.add_action(laser_to_base_link_tf)
    ld.add_action(robot_state_publisher)

    # Add localization nodes
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    # Add visualization
    ld.add_action(rviz_node)

    return ld

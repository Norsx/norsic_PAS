from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node
from launch.substitutions import Command, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Putanje do potrebnih datoteka
    urdf_file = PathJoinSubstitution(
        [FindPackageShare('fanuc_m710ic_support'), 'urdf', 'm710ic50.xacro']
    )
    controllers_file = PathJoinSubstitution(
        [FindPackageShare('fanuc_m710ic_support'), 'config', 'fanuc_controllers.yaml']
    )
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('fanuc_m710ic_support'), 'rviz', 'view_robot.rviz']
    )
    
    # Robot description
    robot_description_content = Command(['xacro ', urdf_file])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    # Controller manager node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[{'robot_description': robot_description}, controllers_file],
        output="both",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{'robot_description': robot_description}],
    )

    # RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    # Spawner za joint_state_broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # Spawner za forward_position_controller
    forward_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "--controller-manager", "/controller_manager"],
    )

    # Spawner za joint_trajectory_controller (učitava se ali ostaje neaktivan)
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_trajectory_controller",
            "--controller-manager", "/controller_manager",
            "--inactive"
        ],
    )

    # Registracija event handlera za sekvencionalno pokretanje
    delayed_forward_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[forward_position_controller_spawner],
        )
    )

    delayed_joint_trajectory_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=forward_position_controller_spawner,
            on_exit=[joint_trajectory_controller_spawner],
        )
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        rviz_node,
        joint_state_broadcaster_spawner,
        delayed_forward_controller_spawner,
        delayed_joint_trajectory_controller_spawner,
    ])

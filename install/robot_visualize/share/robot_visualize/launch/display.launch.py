import launch
from launch.substitutions import Command, LaunchConfiguration
import launch_ros
import os
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkgPath = launch_ros.substitutions.FindPackageShare(package='robot_visualize').find('robot_visualize')
    urdfPath = launch_ros.substitutions.FindPackageShare(package='fanuc_m710ic_support').find('fanuc_m710ic_support')
    xacroModelPath = os.path.join(urdfPath, 'urdf/m710ic50.xacro')
    rvizConfigPath = os.path.join(pkgPath, 'rviz/config.rviz')

    # Pretvori XACRO u URDF koristeći Command i wrapper za string
    robot_description = ParameterValue(
        Command(['xacro ', xacroModelPath]),
        value_type=str
    )

    params = {'robot_description': robot_description}

    robot_state_publisher_node = launch_ros.actions.Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )

    joint_state_publisher_node = launch_ros.actions.Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[params],
        condition=launch.conditions.UnlessCondition(LaunchConfiguration('gui'))
    )

    joint_state_publisher_gui_node = launch_ros.actions.Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        parameters=[params],
        condition=launch.conditions.IfCondition(LaunchConfiguration('gui'))
    )

    rviz_node = launch_ros.actions.Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rvizConfigPath]
    )

    return launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='True',
            description='Flag to enable joint_state_publisher_gui instead of joint_state_publisher'
        ),
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    
    # Dohvaćanje putanje do paketa
    pkg_share = FindPackageShare('fanuc_m710ic_support')
  
    # Definiranje putanje do URDF/XACRO datoteke
    default_urdf_model_path = PathJoinSubstitution(
        [pkg_share, 'urdf', 'm710ic50.xacro']
    )
    
    # Definiranje putanje do RViz konfiguracije (opcionalno)
    default_rviz_config_path = PathJoinSubstitution(
        [pkg_share, 'rviz', 'view_robot.rviz']
    )
    
    # Launch argumenti
    urdf_model = LaunchConfiguration('urdf_model')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # Deklaracija launch argumenata
    declare_urdf_model_cmd = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_model_path,
        description='Apsolutna putanja do URDF datoteke robota'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true',
        description='Pokreće li se RViz'
    )
    
    declare_rviz_config_cmd = DeclareLaunchArgument(
        name='rviz_config',
        default_value=default_rviz_config_path,
        description='Putanja do RViz konfiguracijske datoteke'
    )
    
    # Obrada URDF/XACRO datoteke
    robot_description_content = Command(['xacro ', urdf_model])
    robot_description = ParameterValue(robot_description_content, value_type=str)
    
    # Robot State Publisher čvor
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )
    
    # Joint State Publisher GUI čvor
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )
    
    # RViz2 čvor
    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config]
    )
    
    # Kreiranje Launch Description
    ld = LaunchDescription()
    
    # Dodavanje deklaracija argumenata
    ld.add_action(declare_urdf_model_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_cmd)
    
    # Dodavanje čvorova
    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)
    ld.add_action(rviz_node)
    
    return ld

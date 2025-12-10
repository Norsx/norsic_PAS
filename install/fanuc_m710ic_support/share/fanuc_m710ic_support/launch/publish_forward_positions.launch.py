from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    publisher = Node(
        package='fanuc_m710ic_support',
        executable='forward_position_publisher.py',
        name='forward_position_publisher',
        output='screen'
    )
    return LaunchDescription([
        publisher
    ])

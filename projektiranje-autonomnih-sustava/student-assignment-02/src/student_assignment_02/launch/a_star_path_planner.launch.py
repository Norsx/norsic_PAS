from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file za A* path planning node"""
    
    # Deklaracija argumenata
    goal_x = DeclareLaunchArgument(
        'goal_x',
        default_value='5.0',
        description='Goal X koordinata (m)'
    )
    goal_y = DeclareLaunchArgument(
        'goal_y',
        default_value='5.0',
        description='Goal Y koordinata (m)'
    )
    start_x = DeclareLaunchArgument(
        'start_x',
        default_value='0.0',
        description='Start X koordinata (m)'
    )
    start_y = DeclareLaunchArgument(
        'start_y',
        default_value='0.0',
        description='Start Y koordinata (m)'
    )
    allow_diagonal = DeclareLaunchArgument(
        'allow_diagonal',
        default_value='true',
        description='Dozvoli dijagonalno kretanje'
    )
    inflation_radius = DeclareLaunchArgument(
        'inflation_radius',
        default_value='1',
        description='Inflation radius oko prepreka (stanice)'
    )
    
    # A* Path Planner čvor
    a_star_node = Node(
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
            }
        ],
        remappings=[]
    )
    
    ld = LaunchDescription([
        goal_x,
        goal_y,
        start_x,
        start_y,
        allow_diagonal,
        inflation_radius,
        a_star_node,
    ])
    
    return ld

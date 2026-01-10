from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Launch file za D* path planning node"""
    
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
    inflation_distance_m = DeclareLaunchArgument(
        'inflation_distance_m',
        default_value='0.5',
        description='Inflation buffer distance u metrima (0.5m = 50cm)'
    )
    max_iterations = DeclareLaunchArgument(
        'max_iterations',
        default_value='50000',
        description='Maksimalan broj iteracija D* algoritma'
    )
    
    # D* Path Planner čvor
    d_star_node = Node(
        package='student_assignment_02',
        executable='d_star_path_planner',
        name='d_star_path_planner',
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
                'max_iterations': LaunchConfiguration('max_iterations'),
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
        inflation_distance_m,
        max_iterations,
        d_star_node,
    ])
    
    return ld

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    pkg_share = FindPackageShare('gps_waypoint_nav')
    
    # 声明参数
    gps_file_arg = DeclareLaunchArgument(
        'gps_file',
        default_value=PathJoinSubstitution([pkg_share, 'data', 'orchard_path.json']),
        description='Path to GPS waypoint file'
    )
    
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'gps_waypoint_params.yaml']),
        description='Path to parameters file'
    )
    
    # GPS路径发布节点
    gps_publisher_node = Node(
        package='gps_waypoint_nav',
        executable='gps_path_publisher',
        name='gps_path_publisher',
        output='screen',
        parameters=[LaunchConfiguration('params_file')],
        remappings=[
            ('gps_file', LaunchConfiguration('gps_file'))
        ]
    )
    
    return LaunchDescription([
        gps_file_arg,
        params_file_arg,
        gps_publisher_node
    ])

# ros2 launch bevnet_nav2_bringup orchard_navigation.launch.py
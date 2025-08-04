from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # 声明参数
    mcap_file_arg = DeclareLaunchArgument(
        'mcap_file',
        default_value='/workspace/bevnet_nav2_ws/data/mcap/rosbag2_2025_07_30-11_09_54_0.mcap',
        description='Path to MCAP file'
    )
    
    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate (1.0 = normal speed)'
    )
    
    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='true',
        description='Loop playback'
    )
    
    # MCAP Player 节点
    mcap_player_node = Node(
        package='mcap_replay',
        executable='mcap_player',  # 不带 .py
        name='mcap_player',
        parameters=[{
            'mcap_file': LaunchConfiguration('mcap_file'),
            'rate': LaunchConfiguration('rate'),
            'loop': LaunchConfiguration('loop'),
            'start_paused': False
        }],
        output='screen'
    )
    
    return LaunchDescription([
        mcap_file_arg,
        rate_arg,
        loop_arg,
        mcap_player_node
    ])

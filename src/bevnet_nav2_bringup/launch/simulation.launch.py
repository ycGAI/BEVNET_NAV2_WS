from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    # 声明参数
    mcap_file_arg = DeclareLaunchArgument(
        'mcap_file',
        default_value='',
        description='Path to MCAP file'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to BEVNet model'
    )
    
    # MCAP播放器
    mcap_player = Node(
        package='mcap_replay',
        executable='mcap_player',
        name='mcap_player',
        parameters=[{
            'mcap_file': LaunchConfiguration('mcap_file'),
            'loop': True,
            'rate': 1.0
        }]
    )
    
    # BEVNet推理节点
    bevnet_node = Node(
        package='bevnet_nav2_core',
        executable='bevnet_inference_node',
        name='bevnet_inference',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'device': 'cuda',
            'visualize': True
        }]
    )
    
    # Nav2
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('bevnet_nav2_bringup'),
                'launch',
                'bevnet_nav2.launch.py'
            ])
        ])
    )
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bevnet_nav2_bringup'),
            'config',
            'rviz_config.rviz'
        ])]
    )
    
    return LaunchDescription([
        mcap_file_arg,
        model_path_arg,
        mcap_player,
        bevnet_node,
        nav2_bringup,
        rviz_node
    ])
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 参数
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    rviz_config = LaunchConfiguration('rviz_config')
    
    # 默认RViz配置
    default_rviz_config = PathJoinSubstitution([
        FindPackageShare('bevnet_nav2_bringup'),
        'config',
        'rviz_config.rviz'
    ])
    
    # RViz节点
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    # BEV可视化工具节点
    bev_visualizer = Node(
        package='bevnet_nav2_core',
        executable='bev_visualizer',
        name='bev_visualizer',
        parameters=[{
            'use_sim_time': use_sim_time,
            'publish_rate': 10.0
        }],
        output='screen'
    )
    
    # 静态TF发布器（如果需要）
    static_tf_publisher = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=default_rviz_config,
            description='Full path to the RViz config file'
        ),
        
        rviz_node,
        bev_visualizer,
        static_tf_publisher
    ])
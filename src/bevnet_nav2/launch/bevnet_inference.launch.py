import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 声明参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/workspace/models/kitti4_single_model.pth',
        description='Path to the BEVNet model weights'
    )
    
    model_type_arg = DeclareLaunchArgument(
        'model_type',
        default_value='single',
        description='Model type: single or recurrent'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run inference on: cuda or cpu'
    )
    
    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/velodyne_points',
        description='Input pointcloud topic'
    )
    
    visualize_arg = DeclareLaunchArgument(
        'visualize',
        default_value='true',
        description='Whether to publish visualization'
    )
    
    # BEVNet推理节点
    bevnet_node = Node(
        package='bevnet_nav2',
        executable='bevnet_inference_node.py',
        name='bevnet_inference',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'model_type': LaunchConfiguration('model_type'),
            'device': LaunchConfiguration('device'),
            'pointcloud_topic': LaunchConfiguration('pointcloud_topic'),
            'visualize': LaunchConfiguration('visualize'),
            'publish_rate': 10.0,
        }],
        remappings=[
            ('/bevnet/costmap', '/local_costmap/costmap'),
        ]
    )
    
    # 可视化节点（可选）
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(get_package_share_directory('bevnet_nav2'), 'rviz', 'bevnet_visualization.rviz')],
        condition=LaunchConfiguration('visualize')
    )
    
    return LaunchDescription([
        model_path_arg,
        model_type_arg,
        device_arg,
        pointcloud_topic_arg,
        visualize_arg,
        bevnet_node,
        # rviz_node,  # 取消注释以启动RViz
    ])
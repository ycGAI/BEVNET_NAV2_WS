#!/usr/bin/env python3
"""
完整的BEVNet + GPS导航系统启动文件
整合数据播放、BEVNet推理、Nav2导航和GPS路径跟随
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    # 声明启动参数
    kitti_dir_arg = DeclareLaunchArgument(
        'kitti_dir',
        default_value='/workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1',
        description='KITTI数据目录'
    )
    
    sequence_arg = DeclareLaunchArgument(
        'sequence',
        default_value='train',
        description='数据序列 (train/valid)'
    )
    
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/workspace/bevnet_nav2_ws/models/best.pth.34',
        description='BEVNet模型路径'
    )
    
    use_gps_arg = DeclareLaunchArgument(
        'use_gps',
        default_value='true',
        description='是否使用GPS路径跟随'
    )
    
    # 1. KITTI数据播放器
    kitti_player = Node(
        package='kitti_player',
        executable='kitti_player_normalized.py',
        name='kitti_player',
        parameters=[{
            'kitti_dir': LaunchConfiguration('kitti_dir'),
            'sequence': LaunchConfiguration('sequence'),
            'rate': 10.0,
            'loop': True,
        }],
        output='screen'
    )
    
    # 2. BEVNet推理节点
    bevnet_node = ExecuteProcess(
        cmd=[
            'python3',
            '/workspace/bevnet_nav2_ws/src/bevnet_nav2_core/bevnet_nav2_core/bevnet_inference_node.py',
            LaunchConfiguration('model_path')
        ],
        output='screen',
        shell=False,
        env={'PYTHONPATH': '/workspace/bevnet:/workspace/bevnet/bevnet:${PYTHONPATH}'}
    )
    
    # 3. Nav2导航栈
    nav2_launch = ExecuteProcess(
        cmd=[
            'ros2', 'launch', 'nav2_bringup', 'navigation_launch.py',
            'use_sim_time:=False',
            'params_file:=/workspace/bevnet_nav2_ws/config/nav2_params.yaml'
        ],
        output='screen'
    )
    
    # 4. GPS路径跟随（可选）
    gps_follower = Node(
        package='gps_waypoint_nav',
        executable='gps_waypoint_follower.py',
        name='gps_waypoint_follower',
        parameters=[{
            'poses_file': [LaunchConfiguration('kitti_dir'), '/sequences/', 
                          LaunchConfiguration('sequence'), '/poses.txt'],
            'waypoint_spacing': 5.0,
            'goal_tolerance': 2.0,
            'loop': True,
            'skip_initial': 10
        }],
        condition=IfCondition(LaunchConfiguration('use_gps')),
        output='screen'
    )
    
    # 5. RViz2可视化
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', '/workspace/bevnet_nav2_ws/config/navigation.rviz'],
        output='screen'
    )
    
    return LaunchDescription([
        # 参数声明
        kitti_dir_arg,
        sequence_arg,
        model_path_arg,
        use_gps_arg,
        
        # 节点启动
        kitti_player,
        bevnet_node,
        nav2_launch,
        gps_follower,
        rviz_node,
    ])
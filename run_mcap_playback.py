#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import sys
import os

# 添加mcap_replay到Python路径
sys.path.insert(0, '/workspace/bevnet_nav2_ws/src')

def main():
    # 初始化ROS2
    rclpy.init(args=sys.argv)
    
    # 导入MCAPPlayer
    from mcap_replay.mcap_player import MCAPPlayer
    
    # 创建节点 - 构造函数会自动处理参数
    player = MCAPPlayer()
    
    # 检查是否加载了文件
    if not player.mcap_file:
        player.get_logger().error('No MCAP file specified!')
        player.get_logger().info('Setting MCAP file manually...')
        
        # 手动设置参数
        player.mcap_file = '/workspace/bevnet_nav2_ws/data/mcap/rosbag2_2025_07_30-11_09_54_0.mcap'
        player.loop = True
        player.rate = 1.0
        player.paused = False
        
        # 重新加载数据
        player.load_mcap_data()
        
        # 重启播放线程
        import threading
        player.playback_thread = threading.Thread(target=player.playback_loop)
        player.playback_thread.daemon = True
        player.playback_thread.start()
    
    player.get_logger().info('MCAP playback starting...')
    
    try:
        rclpy.spin(player)
    except KeyboardInterrupt:
        player.get_logger().info('Stopping playback...')
    finally:
        player.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    # 设置命令行参数
    sys.argv = [
        'run_mcap_playback.py',
        '--ros-args',
        '-p', 'mcap_file:=/workspace/bevnet_nav2_ws/data/mcap/rosbag2_2025_07_30-11_09_54_0.mcap',
        '-p', 'loop:=true',
        '-p', 'rate:=1.0',
        '-p', 'start_paused:=false'
    ]
    
    main()

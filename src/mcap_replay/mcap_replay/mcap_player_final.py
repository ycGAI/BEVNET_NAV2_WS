#!/usr/bin/env python3
import sys
import time
import threading
import argparse
import rclpy
from rclpy.node import Node
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages
from rosidl_runtime_py.utilities import get_message
from tf2_ros import TransformBroadcaster

class MCAPPlayerFixed(Node):
    def __init__(self, mcap_file, rate=1.0, loop=True):
        super().__init__('mcap_player_fixed')
        
        self.mcap_file = mcap_file
        self.rate = rate
        self.loop = loop
        self.topic_publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.messages = []
        self.paused = False
        
        # 加载MCAP文件
        self.load_mcap()
        
        # 打印信息
        self.get_logger().info('='*40)
        self.get_logger().info(f'MCAP Player Ready')
        self.get_logger().info(f'File: {self.mcap_file}')
        self.get_logger().info(f'Messages: {len(self.messages)}')
        self.get_logger().info(f'Rate: {self.rate}x')
        self.get_logger().info(f'Loop: {self.loop}')
        self.get_logger().info('='*40)
        
        # 启动播放线程
        if self.messages:
            self.play_thread = threading.Thread(target=self.play_loop)
            self.play_thread.daemon = True
            self.play_thread.start()
        
    def load_mcap(self):
        self.get_logger().info(f'Loading MCAP file...')
        
        with open(self.mcap_file, 'rb') as f:
            reader = make_reader(f)
            summary = reader.get_summary()
            
            self.get_logger().info(f'Found {len(summary.channels)} topics:')
            for channel in summary.channels.values():
                self.get_logger().info(f'  - {channel.topic}')
            
            # 读取所有消息
            f.seek(0)
            topic_counts = {}
            
            for msg in read_ros2_messages(f):
                topic = msg.channel.topic
                
                # 统计
                if topic not in topic_counts:
                    topic_counts[topic] = 0
                topic_counts[topic] += 1
                
                # 创建发布器（跳过tf，因为用tf_broadcaster）
                if topic not in self.topic_publishers and '/tf' not in topic:
                    try:
                        msg_type = get_message(msg.schema.name)
                        self.topic_publishers[topic] = self.create_publisher(msg_type, topic, 10)
                        self.get_logger().info(f'Created publisher: {topic}')
                    except Exception as e:
                        self.get_logger().debug(f'Cannot create publisher for {topic}: {e}')
                
                # 转换时间戳
                if hasattr(msg.log_time, 'total_seconds'):
                    # 如果是timedelta，转换为纳秒
                    timestamp_ns = int(msg.log_time.total_seconds() * 1e9)
                elif hasattr(msg, 'log_time_ns'):
                    timestamp_ns = msg.log_time_ns
                else:
                    timestamp_ns = int(msg.log_time)
                
                # 存储消息
                self.messages.append({
                    'topic': topic,
                    'time': timestamp_ns,  # 纳秒
                    'data': msg.ros_msg,
                    'schema': msg.schema.name
                })
            
            # 排序
            self.messages.sort(key=lambda x: x['time'])
            
            # 打印统计
            self.get_logger().info(f'Loaded {len(self.messages)} messages')
            for topic, count in sorted(topic_counts.items()):
                self.get_logger().info(f'  {topic}: {count}')
    
    def publish_message(self, msg_data):
        """发布单条消息"""
        topic = msg_data['topic']
        data = msg_data['data']
        
        try:
            # TF消息特殊处理
            if '/tf' in topic and hasattr(data, 'transforms'):
                for transform in data.transforms:
                    self.tf_broadcaster.sendTransform(transform)
            elif topic in self.topic_publishers:
                self.topic_publishers[topic].publish(data)
        except Exception as e:
            self.get_logger().debug(f'Error publishing to {topic}: {e}')
    
    def play_loop(self):
        """播放循环"""
        self.get_logger().info('Starting playback...')
        
        while rclpy.ok():
            if self.paused:
                time.sleep(0.1)
                continue
            
            if not self.messages:
                break
            
            # 获取时间基准
            start_time = self.messages[0]['time']
            play_start = time.time()
            
            # 播放所有消息
            for i, msg in enumerate(self.messages):
                if not rclpy.ok() or self.paused:
                    break
                
                # 时间同步
                msg_time = (msg['time'] - start_time) / 1e9  # 转换为秒
                elapsed = time.time() - play_start
                wait_time = (msg_time / self.rate) - elapsed
                
                if wait_time > 0:
                    time.sleep(wait_time)
                
                # 发布消息
                self.publish_message(msg)
                
                # 定期打印进度
                if i % 1000 == 0:
                    progress = (i / len(self.messages)) * 100
                    self.get_logger().info(f'Progress: {progress:.1f}% ({i}/{len(self.messages)})')
            
            self.get_logger().info('Playback complete')
            
            if not self.loop:
                break
            else:
                self.get_logger().info('Looping in 3 seconds...')
                time.sleep(3)
        
        self.get_logger().info('Playback ended')

def main():
    parser = argparse.ArgumentParser(description='MCAP Player for ROS2')
    parser.add_argument('mcap_file', nargs='?', 
                       default='/workspace/data/raw_demo_rosbag/row_coverage_1_0.mcap',
                       help='Path to MCAP file')
    parser.add_argument('--rate', '-r', type=float, default=1.0, 
                       help='Playback rate')
    parser.add_argument('--loop', '-l', action='store_true', 
                       help='Loop playback')
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        player = MCAPPlayerFixed(args.mcap_file, args.rate, args.loop)
        rclpy.spin(player)
    except KeyboardInterrupt:
        print('\nStopping...')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
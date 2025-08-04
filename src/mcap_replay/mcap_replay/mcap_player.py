#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages
import time
import threading

class MCAPPlayer(Node):
    def __init__(self):
        super().__init__('mcap_player')
        
        # 参数
        self.declare_parameter('mcap_file', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('start_paused', False)
        
        self.mcap_file = self.get_parameter('mcap_file').value
        self.loop = self.get_parameter('loop').value
        self.rate = self.get_parameter('rate').value
        self.paused = self.get_parameter('start_paused').value
        
        # 发布器
        self.publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 加载数据
        self.load_mcap_data()
        
        # 回放控制
        self.playback_thread = threading.Thread(target=self.playback_loop)
        self.playback_thread.start()
        
        self.get_logger().info(f'MCAP player initialized with file: {self.mcap_file}')
    
    def load_mcap_data(self):
        """加载MCAP文件"""
        self.messages = []
        
        with open(self.mcap_file, 'rb') as f:
            reader = make_reader(f)
            
            # 读取所有消息
            for schema, channel, message, ros_msg in read_ros2_messages(reader):
                # 创建对应的发布器
                if channel.topic not in self.publishers:
                    msg_type = type(ros_msg)
                    self.publishers[channel.topic] = self.create_publisher(
                        msg_type, channel.topic, 10)
                
                self.messages.append({
                    'timestamp': message.log_time,
                    'topic': channel.topic,
                    'msg': ros_msg
                })
        
        # 按时间排序
        self.messages.sort(key=lambda x: x['timestamp'])
        self.get_logger().info(f'Loaded {len(self.messages)} messages')
    
    def playback_loop(self):
        """回放循环"""
        while rclpy.ok():
            if self.paused:
                time.sleep(0.1)
                continue
                
            start_time = self.messages[0]['timestamp']
            playback_start = time.time()
            
            for msg_data in self.messages:
                if not rclpy.ok() or self.paused:
                    break
                    
                # 计算等待时间
                msg_time = (msg_data['timestamp'] - start_time) / 1e9  # 纳秒转秒
                elapsed = time.time() - playback_start
                wait_time = (msg_time / self.rate) - elapsed
                
                if wait_time > 0:
                    time.sleep(wait_time)
                
                # 发布消息
                self.publish_message(msg_data)
            
            if not self.loop:
                break
                
        self.get_logger().info('Playback finished')
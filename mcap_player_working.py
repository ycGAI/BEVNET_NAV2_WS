#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from tf2_ros import TransformBroadcaster
import time
import threading
import os

class MCAPPlayerWorking(Node):
    def __init__(self):
        super().__init__('mcap_player_working')
        
        self.declare_parameter('mcap_file', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('rate', 1.0)
        
        self.mcap_file = self.get_parameter('mcap_file').value
        self.loop = self.get_parameter('loop').value
        self.rate = self.get_parameter('rate').value
        
        if not os.path.exists(self.mcap_file):
            self.get_logger().error(f'File not found: {self.mcap_file}')
            return
            
        self.publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.messages = []
        
        self.load_mcap()
        
        if self.messages:
            self.thread = threading.Thread(target=self.playback)
            self.thread.daemon = True
            self.thread.start()
    
    def load_mcap(self):
        self.get_logger().info(f'Loading: {self.mcap_file}')
        
        # 使用转换
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=self.mcap_file, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr",
                output_serialization_format="cdr"
            )
        )
        
        topic_types = reader.get_all_topics_and_types()
        type_map = {topic.name: topic.type for topic in topic_types}
        
        self.get_logger().info(f"Found {len(topic_types)} topics")
        for topic in topic_types:
            self.get_logger().info(f"  {topic.name}: {topic.type}")
        
        count = 0
        while reader.has_next() and count < 100000:  # 限制消息数
            try:
                topic, data, timestamp = reader.read_next()
                
                msg_type_name = type_map.get(topic)
                if not msg_type_name:
                    continue
                
                msg_type = get_message(msg_type_name)
                msg = deserialize_message(data, msg_type)
                
                # 创建发布器
                if topic not in self.publishers:
                    self.publishers[topic] = self.create_publisher(msg_type, topic, 10)
                    self.get_logger().info(f'Created publisher: {topic}')
                
                self.messages.append({
                    'topic': topic,
                    'timestamp': timestamp,
                    'data': msg
                })
                count += 1
                
            except Exception as e:
                continue
        
        del reader
        
        self.messages.sort(key=lambda x: x['timestamp'])
        self.get_logger().info(f'Loaded {len(self.messages)} messages')
    
    def playback(self):
        self.get_logger().info('Starting playback...')
        
        while rclpy.ok():
            if not self.messages:
                break
                
            start_time = self.messages[0]['timestamp']
            playback_start = time.time()
            
            for msg in self.messages:
                if not rclpy.ok():
                    break
                    
                msg_time = (msg['timestamp'] - start_time) / 1e9
                elapsed = time.time() - playback_start
                wait = (msg_time / self.rate) - elapsed
                
                if wait > 0:
                    time.sleep(wait)
                
                if msg['topic'] in self.publishers:
                    self.publishers[msg['topic']].publish(msg['data'])
            
            if not self.loop:
                break
            
            self.get_logger().info('Looping...')
            time.sleep(1)

def main():
    rclpy.init()
    node = MCAPPlayerWorking()
    rclpy.spin(node)

if __name__ == '__main__':
    main()

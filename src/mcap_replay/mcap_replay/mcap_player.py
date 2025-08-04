#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage, NavSatFix, LaserScan, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import TransformStamped
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages
import time
import threading
from rosidl_runtime_py.utilities import get_message

class MCAPPlayer(Node):
    def __init__(self):
        super().__init__('mcap_player')
        
        # 参数
        self.declare_parameter('mcap_file', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('start_paused', False)
        self.declare_parameter('topics', [])  # 要播放的话题列表，空表示全部
        
        self.mcap_file = self.get_parameter('mcap_file').value
        self.loop = self.get_parameter('loop').value
        self.rate = self.get_parameter('rate').value
        self.paused = self.get_parameter('start_paused').value
        self.selected_topics = self.get_parameter('topics').value
        
        # 发布器
        self.publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 消息类型映射
        self.msg_type_map = {
            'sensor_msgs/msg/PointCloud2': PointCloud2,
            'sensor_msgs/msg/CompressedImage': CompressedImage,
            'sensor_msgs/msg/NavSatFix': NavSatFix,
            'sensor_msgs/msg/LaserScan': LaserScan,
            'sensor_msgs/msg/CameraInfo': CameraInfo,
            'geometry_msgs/msg/Twist': Twist,
            'nav_msgs/msg/Odometry': Odometry,
            'tf2_msgs/msg/TFMessage': TFMessage,
        }
        
        # 加载数据
        self.messages = []
        self.load_mcap_data()
        
        # 回放控制
        self.playback_thread = threading.Thread(target=self.playback_loop)
        self.playback_thread.daemon = True
        
        self.get_logger().info(f'MCAP player initialized')
        self.get_logger().info(f'File: {self.mcap_file}')
        self.get_logger().info(f'Messages: {len(self.messages)}')
        self.get_logger().info(f'Rate: {self.rate}x')
        self.get_logger().info(f'Loop: {self.loop}')
        
        # 启动回放
        self.playback_thread.start()
    
    def get_msg_type_from_string(self, type_string):
        """从字符串获取消息类型"""
        # 尝试从预定义映射获取
        if type_string in self.msg_type_map:
            return self.msg_type_map[type_string]
        
        # 尝试动态导入
        try:
            return get_message(type_string)
        except Exception as e:
            self.get_logger().warn(f"无法获取消息类型 {type_string}: {e}")
            return None
    
    def load_mcap_data(self):
        """加载MCAP文件"""
        self.get_logger().info(f'加载 MCAP 文件: {self.mcap_file}')
        
        try:
            with open(self.mcap_file, 'rb') as f:
                reader = make_reader(f)
                
                # 统计信息
                topic_count = {}
                
                # 读取所有消息
                for schema, channel, message, ros_msg in read_ros2_messages(reader):
                    topic = channel.topic
                    
                    # 如果指定了话题列表，只处理指定的话题
                    if self.selected_topics and topic not in self.selected_topics:
                        continue
                    
                    # 创建对应的发布器
                    if topic not in self.publishers:
                        # 特殊处理 TF 消息
                        if topic in ['/tf', '/tf_static']:
                            # TF 消息通过 TransformBroadcaster 发布，不需要单独的发布器
                            pass
                        else:
                            msg_type = type(ros_msg)
                            self.publishers[topic] = self.create_publisher(
                                msg_type, topic, 10)
                            self.get_logger().info(f'创建发布器: {topic} ({schema.name})')
                    
                    # 统计话题消息数
                    topic_count[topic] = topic_count.get(topic, 0) + 1
                    
                    self.messages.append({
                        'timestamp': message.log_time,
                        'topic': topic,
                        'msg': ros_msg
                    })
                
                # 按时间排序
                self.messages.sort(key=lambda x: x['timestamp'])
                
                # 打印统计信息
                self.get_logger().info(f'加载完成，共 {len(self.messages)} 条消息')
                for topic, count in sorted(topic_count.items()):
                    self.get_logger().info(f'  {topic}: {count} 条消息')
                    
        except Exception as e:
            self.get_logger().error(f'加载 MCAP 文件失败: {e}')
            import traceback
            traceback.print_exc()
    
    def publish_message(self, msg_data):
        """发布单个消息"""
        topic = msg_data['topic']
        msg = msg_data['msg']
        
        try:
            # 特殊处理 TF 消息
            if topic in ['/tf', '/tf_static']:
                for transform in msg.transforms:
                    self.tf_broadcaster.sendTransform(transform)
            else:
                # 普通消息
                if topic in self.publishers:
                    self.publishers[topic].publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布消息失败 {topic}: {e}')
    
    def playback_loop(self):
        """回放循环"""
        self.get_logger().info('开始回放...')
        
        while rclpy.ok():
            if self.paused:
                time.sleep(0.1)
                continue
            
            if not self.messages:
                self.get_logger().warn('没有消息可以回放')
                break
                
            start_time = self.messages[0]['timestamp']
            playback_start = time.time()
            
            for i, msg_data in enumerate(self.messages):
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
                
                # 定期打印进度
                if i % 100 == 0:
                    progress = (i / len(self.messages)) * 100
                    self.get_logger().info(f'回放进度: {progress:.1f}% ({i}/{len(self.messages)})')
            
            if not self.loop:
                break
            else:
                self.get_logger().info('循环回放...')
                
        self.get_logger().info('回放结束')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        player = MCAPPlayer()
        rclpy.spin(player)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
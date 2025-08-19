#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage, NavSatFix, LaserScan, CameraInfo, Imu
from geometry_msgs.msg import Twist, PoseStamped, TwistWithCovarianceStamped, PolygonStamped, TransformStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.msg import Costmap
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import Log
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from tf2_msgs.msg import TFMessage
import time
import threading
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

class MCAPPlayer(Node):
    def __init__(self):
        super().__init__('mcap_player')
        
        # 声明参数
        self.declare_parameter('mcap_file', '')
        self.declare_parameter('loop', True)
        self.declare_parameter('rate', 1.0)
        self.declare_parameter('start_paused', False)
        self.declare_parameter('topics', [])
        
        # 获取参数值
        self.mcap_file = self.get_parameter('mcap_file').value
        self.loop = self.get_parameter('loop').value
        self.rate = self.get_parameter('rate').value
        self.paused = self.get_parameter('start_paused').value
        self.selected_topics = self.get_parameter('topics').value
        
        # 初始化
        self.topic_publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        self.messages = []
        
        if not self.mcap_file:
            self.get_logger().error('No MCAP file specified!')
            return
            
        # 使用 mcap 库加载数据
        self.load_mcap_data()
        
        # 创建回放线程
        self.playback_thread = threading.Thread(target=self.playback_loop)
        self.playback_thread.daemon = True
        
        # 打印信息
        self.get_logger().info(f'=== MCAP Player Initialized ===')
        self.get_logger().info(f'File: {self.mcap_file}')
        self.get_logger().info(f'Total Messages: {len(self.messages)}')
        self.get_logger().info(f'Playback Rate: {self.rate}x')
        self.get_logger().info(f'Loop Mode: {self.loop}')
        self.get_logger().info(f'================================')
        
        # 启动回放
        if self.messages:
            self.playback_thread.start()
        else:
            self.get_logger().warn('No messages to playback')
    
    def load_mcap_data(self):
        """使用 mcap 库加载数据"""
        self.get_logger().info(f'Loading MCAP file: {self.mcap_file}')
        
        try:
            decoder_factory = DecoderFactory()
            topic_count = {}
            success_count = 0
            error_count = 0
            
            with open(self.mcap_file, 'rb') as f:
                reader = make_reader(f, decoder_factories=[decoder_factory])
                
                # 获取话题信息
                channels = reader.get_summary().channels
                self.get_logger().info(f"Found {len(channels)} topics:")
                for channel_id, channel in channels.items():
                    self.get_logger().info(f"  {channel.topic}: {channel.message_encoding}")
                
                # 读取消息
                for schema, channel, message in reader.iter_messages():
                    try:
                        topic = channel.topic
                        
                        # 如果指定了话题列表，只处理指定的话题
                        if self.selected_topics and topic not in self.selected_topics:
                            continue
                        
                        # 解码消息
                        msg = decoder_factory.decoder_for(channel.message_encoding, schema)(message.data)
                        
                        # 获取消息类型
                        msg_type = type(msg)
                        
                        # 创建发布器（跳过TF，因为会特殊处理）
                        if topic not in self.topic_publishers and 'tf' not in topic.lower():
                            try:
                                # 直接使用解码后的消息类型
                                self.topic_publishers[topic] = self.create_publisher(msg_type, topic, 10)
                                self.get_logger().info(f'Created publisher: {topic}')
                            except Exception as e:
                                # 如果创建失败，跳过这个话题
                                self.get_logger().warn(f'Failed to create publisher for {topic}: {e}')
                                continue
                        
                        # 存储消息
                        self.messages.append({
                            'topic': topic,
                            'timestamp': message.log_time,  # 纳秒
                            'data': msg,
                            'type': str(msg_type)
                        })
                        
                        # 统计
                        if topic not in topic_count:
                            topic_count[topic] = 0
                        topic_count[topic] += 1
                        success_count += 1
                        
                    except Exception as e:
                        error_count += 1
                        if error_count <= 10:
                            self.get_logger().debug(f"Failed to process message: {e}")
                        continue
            
            # 按时间排序
            self.messages.sort(key=lambda x: x['timestamp'])
            
            # 打印统计信息
            self.get_logger().info(f'=== MCAP File Loaded ===')
            self.get_logger().info(f'Successfully loaded: {success_count} messages')
            if error_count > 0:
                self.get_logger().warn(f'Failed/Skipped: {error_count} messages')
            self.get_logger().info(f'Topic statistics:')
            for topic, count in sorted(topic_count.items()):
                self.get_logger().info(f'  {topic}: {count} messages')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load MCAP file: {e}')
            import traceback
            traceback.print_exc()
    
    def publish_message(self, msg_data):
        """发布单条消息"""
        topic = msg_data['topic']
        msg = msg_data['data']
        
        try:
            # 特殊处理TF消息
            if '/tf_static' in topic:
                if hasattr(msg, 'transforms'):
                    for transform in msg.transforms:
                        self.static_tf_broadcaster.sendTransform(transform)
            elif '/tf' in topic:
                if hasattr(msg, 'transforms'):
                    for transform in msg.transforms:
                        self.tf_broadcaster.sendTransform(transform)
            elif topic in self.topic_publishers:
                self.topic_publishers[topic].publish(msg)
        except Exception as e:
            # 静默处理错误，避免刷屏
            pass
    
    def playback_loop(self):
        """回放循环主函数"""
        self.get_logger().info('Starting MCAP data playback...')
        
        while rclpy.ok():
            if self.paused:
                time.sleep(0.1)
                continue
            
            if not self.messages:
                self.get_logger().warn('No messages to playback')
                break
                
            # 获取起始时间戳
            start_time = self.messages[0]['timestamp']
            playback_start = time.time()
            
            # 遍历所有消息
            message_count = 0
            for i, msg_data in enumerate(self.messages):
                if not rclpy.ok() or self.paused:
                    break
                    
                # 计算消息的相对时间
                msg_time = (msg_data['timestamp'] - start_time) / 1e9
                elapsed = time.time() - playback_start
                wait_time = (msg_time / self.rate) - elapsed
                
                # 等待到消息的发布时间
                if wait_time > 0:
                    time.sleep(wait_time)
                
                # 发布消息
                self.publish_message(msg_data)
                message_count += 1
                
                # 定期打印进度
                if i % 500 == 0 and i > 0:
                    progress = (i / len(self.messages)) * 100
                    self.get_logger().info(f'Progress: {progress:.1f}% ({i}/{len(self.messages)})')
            
            # 完成一轮播放
            self.get_logger().info(f'Completed one round, published {message_count} messages')
            
            # 检查是否循环播放
            if not self.loop:
                self.get_logger().info('Playback completed')
                break
            else:
                self.get_logger().info('Loop playback, restarting in 3 seconds...')
                time.sleep(3.0)
                
        self.get_logger().info('Playback ended')

def main(args=None):
    rclpy.init(args=args)
    
    try:
        player = MCAPPlayer()
        rclpy.spin(player)
    except KeyboardInterrupt:
        print('\nUser interrupted, stopping...')
    except Exception as e:
        print(f'Error occurred: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
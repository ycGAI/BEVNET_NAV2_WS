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
from mcap_ros2.decoder import DecoderFactory
import time
import threading
from rosidl_runtime_py.utilities import get_message

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
        
        # 消息类型映射
        self.msg_type_map = {
            'sensor_msgs/msg/PointCloud2': PointCloud2,
            'sensor_msgs/msg/CompressedImage': CompressedImage,
            'sensor_msgs/msg/NavSatFix': NavSatFix,
            'sensor_msgs/msg/LaserScan': LaserScan,
            'sensor_msgs/msg/CameraInfo': CameraInfo,
            'geometry_msgs/msg/Twist': Twist,
            'geometry_msgs/msg/PoseStamped': PoseStamped,
            'nav_msgs/msg/Odometry': Odometry,
            'tf2_msgs/msg/TFMessage': TFMessage,
        }
        
        self.messages = []
        
        if not self.mcap_file:
            self.get_logger().error('No MCAP file specified!')
            return
            
        # 加载数据
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
    
    def get_msg_type_from_string(self, type_string):
        """从字符串获取消息类型"""
        if type_string in self.msg_type_map:
            return self.msg_type_map[type_string]
        try:
            return get_message(type_string)
        except Exception as e:
            self.get_logger().warn(f"无法获取消息类型 {type_string}: {e}")
            return None
    
    def load_mcap_data(self):
        """加载MCAP文件数据"""
        self.get_logger().info(f'正在加载 MCAP 文件: {self.mcap_file}')
        
        try:
            with open(self.mcap_file, 'rb') as f:
                decoder_factory = DecoderFactory()
                reader = make_reader(f, decoder_factories=[decoder_factory])
                
                topic_count = {}
                error_count = 0
                success_count = 0
                
                # 读取所有消息
                for ros2_msg in read_ros2_messages(reader):
                    try:
                        # 从McapROS2Message对象获取属性
                        topic = ros2_msg.channel.topic
                        
                        # 获取时间戳 - 使用publish_time_ns (纳秒)
                        timestamp = ros2_msg.publish_time_ns if hasattr(ros2_msg, 'publish_time_ns') else ros2_msg.publish_time
                        
                        # 如果timestamp不是整数，转换为纳秒
                        if not isinstance(timestamp, (int, float)):
                            # 可能是datetime对象，转换为纳秒
                            timestamp = int(timestamp.timestamp() * 1e9) if hasattr(timestamp, 'timestamp') else 0
                        
                        ros_msg = ros2_msg.ros_msg
                        
                        # 从schema获取消息类型
                        message_type = ros2_msg.schema.name if ros2_msg.schema else None
                        
                        if not message_type:
                            self.get_logger().warn(f"无法获取消息类型，跳过话题: {topic}")
                            error_count += 1
                            continue
                        
                        # 如果指定了话题列表，只处理指定的话题
                        if self.selected_topics and topic not in self.selected_topics:
                            continue
                        
                        # 创建发布器
                        if topic not in self.topic_publishers:
                            if message_type and 'TFMessage' in message_type:
                                self.get_logger().info(f'话题 {topic} 将使用 TF Broadcaster')
                            else:
                                msg_type = self.get_msg_type_from_string(message_type)
                                if msg_type:
                                    self.topic_publishers[topic] = self.create_publisher(
                                        msg_type, topic, 10)
                                    self.get_logger().info(f'创建发布器: {topic} [{message_type}]')
                                else:
                                    self.get_logger().warn(f'无法为话题 {topic} 创建发布器，消息类型: {message_type}')
                        
                        # 存储消息
                        self.messages.append({
                            'topic': topic,
                            'timestamp': timestamp,  # 纳秒
                            'data': ros_msg,
                            'type': message_type
                        })
                        
                        # 统计
                        if topic not in topic_count:
                            topic_count[topic] = 0
                        topic_count[topic] += 1
                        success_count += 1
                        
                    except UnicodeDecodeError as e:
                        # 跳过无法解码的消息
                        error_count += 1
                        if error_count <= 5:
                            self.get_logger().warn(f"UTF-8解码错误，跳过消息: {e}")
                        continue
                    except Exception as e:
                        error_count += 1
                        if error_count <= 10:
                            self.get_logger().warn(f"处理消息时出错: {e}")
                        continue
                
                # 按时间排序
                self.messages.sort(key=lambda x: x['timestamp'])
                
                # 打印统计信息
                self.get_logger().info(f'=== MCAP 文件加载完成 ===')
                self.get_logger().info(f'成功加载: {success_count} 条消息')
                if error_count > 0:
                    self.get_logger().warn(f'失败/跳过: {error_count} 条消息')
                self.get_logger().info(f'话题统计:')
                for topic, count in sorted(topic_count.items()):
                    self.get_logger().info(f'  {topic}: {count} 条')
                self.get_logger().info(f'========================')
                    
        except FileNotFoundError:
            self.get_logger().error(f'找不到文件: {self.mcap_file}')
        except Exception as e:
            self.get_logger().error(f'加载MCAP文件失败: {e}')
            # 如果已经加载了一些消息，继续使用
            if self.messages:
                self.get_logger().info(f'已经成功加载 {len(self.messages)} 条消息，将继续播放')
            else:
                import traceback
                traceback.print_exc()
    
    def publish_message(self, msg_data):
        """发布单条消息"""
        topic = msg_data['topic']
        msg = msg_data['data']
        msg_type = msg_data['type']
        
        try:
            # 特殊处理TF消息
            if msg_type and 'TFMessage' in msg_type:
                if hasattr(msg, 'transforms'):
                    for transform in msg.transforms:
                        self.tf_broadcaster.sendTransform(transform)
            elif topic in self.topic_publishers:
                self.topic_publishers[topic].publish(msg)
        except Exception as e:
            self.get_logger().error(f'发布消息失败 {topic}: {e}')
    
    def playback_loop(self):
        """回放循环主函数"""
        self.get_logger().info('开始回放 MCAP 数据...')
        
        while rclpy.ok():
            if self.paused:
                time.sleep(0.1)
                continue
            
            if not self.messages:
                self.get_logger().warn('没有消息可以回放')
                break
                
            # 获取起始时间戳（纳秒）
            start_time = self.messages[0]['timestamp']
            playback_start = time.time()
            
            # 遍历所有消息
            for i, msg_data in enumerate(self.messages):
                if not rclpy.ok() or self.paused:
                    break
                    
                # 计算消息的相对时间（秒）
                msg_time = (msg_data['timestamp'] - start_time) / 1e9  # 纳秒转秒
                elapsed = time.time() - playback_start
                wait_time = (msg_time / self.rate) - elapsed
                
                # 等待到消息的发布时间
                if wait_time > 0:
                    time.sleep(wait_time)
                
                # 发布消息
                self.publish_message(msg_data)
                
                # 定期打印进度
                if i % 10 == 0:  # 每10条消息打印一次
                    progress = (i / len(self.messages)) * 100
                    self.get_logger().info(f'回放进度: {progress:.1f}% ({i}/{len(self.messages)})')
            
            # 完成一轮播放
            self.get_logger().info('完成一轮播放')
            
            # 检查是否循环播放
            if not self.loop:
                self.get_logger().info('回放完成')
                break
            else:
                self.get_logger().info('循环回放，3秒后重新开始...')
                time.sleep(3.0)  # 循环之间暂停3秒
                
        self.get_logger().info('回放结束')

def main(args=None):
    """主函数"""
    rclpy.init(args=args)
    
    try:
        player = MCAPPlayer()
        rclpy.spin(player)
    except KeyboardInterrupt:
        print('\n用户中断，正在停止...')
    except Exception as e:
        print(f'发生错误: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# pip3 install --upgrade mcap mcap-ros2-support
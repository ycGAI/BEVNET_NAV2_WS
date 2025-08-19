#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, CompressedImage, NavSatFix, LaserScan, CameraInfo, Imu
from geometry_msgs.msg import Twist, PoseStamped, TwistWithCovarianceStamped, PolygonStamped
from nav_msgs.msg import Odometry, OccupancyGrid
from diagnostic_msgs.msg import DiagnosticArray
from nav2_msgs.msg import Costmap
from std_msgs.msg import String, Bool
from rcl_interfaces.msg import Log
from tf2_ros import TransformBroadcaster
from tf2_msgs.msg import TFMessage
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import time
import threading

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
        self.messages = []
        
        if not self.mcap_file:
            self.get_logger().error('No MCAP file specified!')
            return
            
        # 使用rosbag2_py加载数据（像KITTI转换脚本一样）
        self.load_mcap_data_rosbag2()
        
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
    
    def load_mcap_data_rosbag2(self):
        """使用rosbag2_py加载MCAP文件（像KITTI转换脚本一样）"""
        self.get_logger().info(f'正在加载 MCAP 文件: {self.mcap_file}')
        
        try:
            # 创建reader
            reader = rosbag2_py.SequentialReader()
            reader.open(
                rosbag2_py.StorageOptions(uri=self.mcap_file, storage_id="mcap"),
                rosbag2_py.ConverterOptions(
                    input_serialization_format="cdr", 
                    output_serialization_format="cdr"
                ),
            )

            # 获取话题类型映射
            topic_types = reader.get_all_topics_and_types()
            type_map = {topic.name: topic.type for topic in topic_types}
            
            # 打印话题信息
            self.get_logger().info(f"发现 {len(topic_types)} 个话题:")
            for topic in topic_types:
                self.get_logger().info(f"  {topic.name}: {topic.type}")
            
            # 统计信息
            topic_count = {}
            success_count = 0
            error_count = 0
            
            # 读取所有消息
            while reader.has_next():
                try:
                    topic, data, timestamp = reader.read_next()
                    
                    # 如果指定了话题列表，只处理指定的话题
                    if self.selected_topics and topic not in self.selected_topics:
                        continue
                    
                    msg_type_name = type_map.get(topic)
                    if not msg_type_name:
                        continue
                    
                    # 反序列化消息
                    msg_type = get_message(msg_type_name)
                    msg = deserialize_message(data, msg_type)
                    
                    # 创建发布器
                    if topic not in self.topic_publishers:
                        # 特殊处理TF消息
                        if 'tf' in topic.lower() and 'static' not in topic.lower():
                            self.get_logger().info(f'话题 {topic} 将使用 TF Broadcaster')
                        elif 'tf_static' in topic.lower():
                            # 静态TF也使用TF broadcaster
                            self.get_logger().info(f'话题 {topic} 将使用 TF Broadcaster (static)')
                        else:
                            # 创建普通发布器
                            self.topic_publishers[topic] = self.create_publisher(msg_type, topic, 10)
                            self.get_logger().info(f'创建发布器: {topic} [{msg_type_name}]')
                    
                    # 存储消息
                    self.messages.append({
                        'topic': topic,
                        'timestamp': timestamp,  # 纳秒
                        'data': msg,
                        'type': msg_type_name
                    })
                    
                    # 统计
                    if topic not in topic_count:
                        topic_count[topic] = 0
                    topic_count[topic] += 1
                    success_count += 1
                    
                except Exception as e:
                    error_count += 1
                    if error_count <= 10:
                        self.get_logger().warn(f"处理消息失败: {e}")
                    continue
            
            # 关闭reader
            del reader
            
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
            
            # 打印关键话题状态
            if '/velodyne_points' in topic_count:
                self.get_logger().info(f'✓ 点云数据: {topic_count["/velodyne_points"]} 帧')
            if '/odom' in topic_count or '/odometry/filtered/global' in topic_count:
                odom_count = topic_count.get('/odom', 0) + topic_count.get('/odometry/filtered/global', 0)
                self.get_logger().info(f'✓ 里程计数据: {odom_count} 条')
                    
        except Exception as e:
            self.get_logger().error(f'加载MCAP文件失败: {e}')
            import traceback
            traceback.print_exc()
    
    def publish_message(self, msg_data):
        """发布单条消息"""
        topic = msg_data['topic']
        msg = msg_data['data']
        msg_type = msg_data['type']
        
        try:
            # 特殊处理TF消息
            if 'tf' in topic.lower():
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
        self.get_logger().info('开始回放 MCAP 数据...')
        
        while rclpy.ok():
            if self.paused:
                time.sleep(0.1)
                continue
            
            if not self.messages:
                self.get_logger().warn('没有消息可以回放')
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
                if i % 50 == 0:
                    progress = (i / len(self.messages)) * 100
                    self.get_logger().info(f'回放进度: {progress:.1f}% ({i}/{len(self.messages)})')
            
            # 完成一轮播放
            self.get_logger().info(f'完成一轮播放，共发布 {message_count} 条消息')
            
            # 检查是否循环播放
            if not self.loop:
                self.get_logger().info('回放完成')
                break
            else:
                self.get_logger().info('循环回放，3秒后重新开始...')
                time.sleep(3.0)
                
        self.get_logger().info('回放结束')

def main(args=None):
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
#!/usr/bin/env python3
"""
完整的MCAP播放器 - 修复所有已知问题
用于播放点云和其他必要话题
"""
import sys
import time
import argparse
import rclpy
from rclpy.node import Node
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages
from rosidl_runtime_py.utilities import get_message
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from datetime import datetime

class MCAPPlayer(Node):
    def __init__(self, mcap_file, rate=1.0, loop=False):
        super().__init__('mcap_player')
        
        self.mcap_file = mcap_file
        self.rate = rate
        self.loop = loop
        self.topic_publishers = {}
        self.tf_broadcaster = TransformBroadcaster(self)
        self.messages = []
        self.playing = False
        
        # QoS配置 - 兼容BEVNet和RViz2
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 必要的话题列表
        self.essential_topics = [
            '/velodyne_points',     # BEVNet输入
            '/tf',                  # 坐标变换
            '/tf_static',           # 静态坐标变换
            '/odom',                # 里程计
            '/imu/data',            # IMU数据
            '/base/gps_node/fix',   # GPS
            '/rover/gps_node/fix',  # GPS备选
            '/odometry/filtered/global',  # 全局里程计
            '/odometry/filtered/local',   # 局部里程计
            '/odometry/gps'         # GPS里程计
        ]
        
        # 要跳过的话题（会导致问题或不需要）
        self.skip_topics = [
            '/ntrip_client/rtcm',
            '/emergency_stop',
            '/coverage_server/coverage_plan',
            '/plan',
            '/image_raw/compressed',
            '/camera',
            '/diagnostics',
            '/rosout',
            '/battery_state',
            '/cmd_vel',
            '/local_costmap/costmap',
            '/lookahead_point',
            '/robot_description'
        ]
        
        # 加载MCAP文件
        self.load_mcap()
        
        # 打印信息
        self.print_info()
        
        # 启动播放
        if self.messages:
            self.msg_index = 0
            self.start_time = None
            self.base_time = self.messages[0]['time']
            # 创建定时器 - 10ms检查一次
            self.timer = self.create_timer(0.01, self.playback_callback)
            self.playing = True
            self.get_logger().info('Playback ready!')
        else:
            self.get_logger().error('No messages to playback!')
    
    def load_mcap(self):
        """加载MCAP文件"""
        self.get_logger().info(f'Loading MCAP: {self.mcap_file}')
        
        try:
            with open(self.mcap_file, 'rb') as f:
                reader = make_reader(f)
                
                # 获取文件信息
                summary = reader.get_summary()
                if summary:
                    total_messages = summary.statistics.message_count
                    self.get_logger().info(f'Total messages in file: {total_messages}')
                
                # 读取消息
                f.seek(0)
                topic_counts = {}
                processed = 0
                skipped = 0
                loaded = 0
                
                for msg in read_ros2_messages(f):
                    processed += 1
                    
                    # 进度报告
                    if processed % 5000 == 0:
                        self.get_logger().info(f'Processed {processed} messages...')
                    
                    topic = msg.channel.topic
                    
                    # 跳过不需要的话题
                    skip = False
                    for skip_pattern in self.skip_topics:
                        if skip_pattern in topic:
                            skip = True
                            break
                    
                    if skip:
                        skipped += 1
                        continue
                    
                    # 只处理必要话题
                    if topic not in self.essential_topics:
                        skipped += 1
                        continue
                    
                    # 统计
                    topic_counts[topic] = topic_counts.get(topic, 0) + 1
                    
                    # 创建发布器
                    if topic not in self.topic_publishers:
                        try:
                            msg_type = get_message(msg.schema.name)
                            self.topic_publishers[topic] = self.create_publisher(
                                msg_type, topic, self.qos
                            )
                            self.get_logger().info(f'✓ Publisher created: {topic}')
                        except Exception as e:
                            self.get_logger().warn(f'✗ Cannot create publisher for {topic}: {e}')
                            continue
                    
                    # 跳过没有发布器的话题
                    if topic not in self.topic_publishers:
                        continue
                    
                    # 检查消息数据
                    if msg.ros_msg is None:
                        self.get_logger().debug(f'Null message for {topic}')
                        continue
                    
                    # 获取时间戳
                    timestamp_ns = self.extract_timestamp(msg)
                    if timestamp_ns is None:
                        continue
                    
                    # 存储消息
                    self.messages.append({
                        'topic': topic,
                        'time': timestamp_ns,
                        'data': msg.ros_msg
                    })
                    loaded += 1
                
                # 按时间排序
                self.messages.sort(key=lambda x: x['time'])
                
                # 打印加载统计
                self.get_logger().info(f'\n=== Loading Complete ===')
                self.get_logger().info(f'Total processed: {processed}')
                self.get_logger().info(f'Skipped: {skipped}')
                self.get_logger().info(f'Loaded: {loaded}')
                self.get_logger().info(f'\nTopic summary:')
                for topic, count in sorted(topic_counts.items()):
                    self.get_logger().info(f'  {topic}: {count} messages')
                    
        except Exception as e:
            self.get_logger().error(f'Error loading MCAP: {e}')
            import traceback
            traceback.print_exc()
    
    def extract_timestamp(self, msg):
        """从消息中提取时间戳"""
        try:
            if hasattr(msg, 'log_time'):
                if isinstance(msg.log_time, datetime):
                    # datetime对象
                    return int(msg.log_time.timestamp() * 1e9)
                elif hasattr(msg.log_time, 'total_seconds'):
                    # timedelta对象
                    return int(msg.log_time.total_seconds() * 1e9)
                else:
                    # 假设是纳秒
                    return int(msg.log_time)
            elif hasattr(msg, 'log_time_ns'):
                return msg.log_time_ns
            elif hasattr(msg, 'publish_time'):
                if msg.publish_time < 1e10:
                    return int(msg.publish_time * 1e9)
                else:
                    return int(msg.publish_time)
            else:
                # 使用当前时间作为备用
                return int(time.time() * 1e9)
        except Exception as e:
            self.get_logger().debug(f'Error extracting timestamp: {e}')
            return None
    
    def playback_callback(self):
        """定时器回调 - 在主线程中发布消息"""
        if not self.playing:
            return
        
        # 检查是否播放完成
        if self.msg_index >= len(self.messages):
            if self.loop:
                self.msg_index = 0
                self.start_time = None
                self.get_logger().info('=== Looping playback ===')
            else:
                self.playing = False
                self.timer.cancel()
                self.get_logger().info('=== Playback complete ===')
            return
        
        # 初始化播放时间
        if self.start_time is None:
            self.start_time = time.time()
            self.get_logger().info('=== Starting playback ===')
        
        # 计算经过时间
        current_time = time.time()
        elapsed = current_time - self.start_time
        
        # 发布所有到期的消息
        messages_published = 0
        while self.msg_index < len(self.messages):
            msg = self.messages[self.msg_index]
            
            # 计算消息应该发布的时间
            msg_time = (msg['time'] - self.base_time) / 1e9 / self.rate
            
            if msg_time <= elapsed:
                # 发布消息
                self.publish_message(msg)
                messages_published += 1
                self.msg_index += 1
                
                # 进度报告
                if self.msg_index % 500 == 0:
                    progress = 100.0 * self.msg_index / len(self.messages)
                    self.get_logger().info(f'Progress: {progress:.1f}% ({self.msg_index}/{len(self.messages)})')
                
                # 点云特别报告
                if msg['topic'] == '/velodyne_points':
                    if self.msg_index % 50 == 0:  # 每50个点云报告一次
                        data = msg['data']
                        if data and hasattr(data, 'header'):
                            pts = data.width * data.height
                            frame = data.header.frame_id
                            self.get_logger().info(f'✓ PointCloud2: {pts} points, frame={frame}')
            else:
                break
        
        # 调试输出
        if messages_published > 0 and self.msg_index % 100 == 0:
            self.get_logger().debug(f'Published {messages_published} messages in this cycle')
    
    def publish_message(self, msg):
        """发布单条消息"""
        topic = msg['topic']
        data = msg['data']
        
        # 检查数据
        if data is None:
            return
        
        try:
            # TF消息特殊处理
            if '/tf' in topic:
                if hasattr(data, 'transforms') and data.transforms:
                    for transform in data.transforms:
                        self.tf_broadcaster.sendTransform(transform)
                else:
                    # 如果不是标准TF消息格式，尝试直接发布
                    if topic in self.topic_publishers:
                        self.topic_publishers[topic].publish(data)
            elif topic in self.topic_publishers:
                # 普通消息
                self.topic_publishers[topic].publish(data)
                
        except Exception as e:
            # 只在第一次错误时报告
            error_key = f'error_{topic.replace("/", "_")}'
            if not hasattr(self, error_key):
                setattr(self, error_key, True)
                self.get_logger().warn(f'Error publishing {topic}: {e}')
    
    def print_info(self):
        """打印播放器信息"""
        self.get_logger().info('='*60)
        self.get_logger().info('MCAP Player')
        self.get_logger().info('='*60)
        self.get_logger().info(f'File: {self.mcap_file}')
        self.get_logger().info(f'Messages loaded: {len(self.messages)}')
        self.get_logger().info(f'Playback rate: {self.rate}x')
        self.get_logger().info(f'Loop: {self.loop}')
        self.get_logger().info(f'Publishers created: {len(self.topic_publishers)}')
        for topic in self.topic_publishers.keys():
            self.get_logger().info(f'  - {topic}')
        self.get_logger().info('='*60)

def main():
    parser = argparse.ArgumentParser(description='MCAP Player for ROS2')
    parser.add_argument('mcap_file', 
                       nargs='?',
                       default='/workspace/data/gyc/thesis/raw_demo_rosbag/row_coverage_1_0.mcap',
                       help='Path to MCAP file')
    parser.add_argument('--rate', '-r', 
                       type=float, 
                       default=1.0,
                       help='Playback rate (default: 1.0)')
    parser.add_argument('--loop', '-l', 
                       action='store_true',
                       help='Loop playback')
    
    args = parser.parse_args()
    
    # 初始化ROS2
    rclpy.init()
    
    try:
        player = MCAPPlayer(args.mcap_file, args.rate, args.loop)
        rclpy.spin(player)
    except KeyboardInterrupt:
        print('\n=== Stopping playback ===')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# export PYTHONPATH=/usr/local/lib/python3.12/dist-packages:$PYTHONPATH
# python3 src/mcap_replay/mcap_replay/mcap_player_final.py /workspace/data/gyc/thesis/raw_demo_rosbag/row_coverage_1_0.mcap
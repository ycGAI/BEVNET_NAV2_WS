#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import numpy as np

class DataSynchronizer(Node):
    def __init__(self):
        super().__init__('data_synchronizer')
        
        # 参数
        self.declare_parameter('sync_queue_size', 10)
        self.declare_parameter('sync_slop', 0.1)
        self.declare_parameter('publish_rate', 10.0)
        
        queue_size = self.get_parameter('sync_queue_size').value
        slop = self.get_parameter('sync_slop').value
        
        # 订阅器
        self.pc_sub = Subscriber(self, PointCloud2, '/velodyne_points')
        self.img_sub = Subscriber(self, Image, '/camera/image_raw')
        self.odom_sub = Subscriber(self, Odometry, '/odom')
        
        # 同步器
        self.sync = ApproximateTimeSynchronizer(
            [self.pc_sub, self.img_sub, self.odom_sub],
            queue_size,
            slop
        )
        self.sync.registerCallback(self.sync_callback)
        
        # 发布器
        self.pc_pub = self.create_publisher(PointCloud2, '/synced/pointcloud', 10)
        self.img_pub = self.create_publisher(Image, '/synced/image', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/synced/pose', 10)
        
        # 统计
        self.sync_count = 0
        self.last_sync_time = self.get_clock().now()
        
        # 定时器用于统计
        self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Data synchronizer initialized')
    
    def sync_callback(self, pc_msg, img_msg, odom_msg):
        """同步回调"""
        # 使用统一的时间戳
        sync_stamp = self.get_clock().now().to_msg()
        
        # 更新消息时间戳
        pc_msg.header.stamp = sync_stamp
        img_msg.header.stamp = sync_stamp
        
        # 创建PoseStamped消息
        pose_msg = PoseStamped()
        pose_msg.header.stamp = sync_stamp
        pose_msg.header.frame_id = odom_msg.header.frame_id
        pose_msg.pose = odom_msg.pose.pose
        
        # 发布同步后的消息
        self.pc_pub.publish(pc_msg)
        self.img_pub.publish(img_msg)
        self.pose_pub.publish(pose_msg)
        
        self.sync_count += 1
    
    def print_stats(self):
        """打印统计信息"""
        current_time = self.get_clock().now()
        duration = (current_time - self.last_sync_time).nanoseconds / 1e9
        
        if duration > 0:
            sync_rate = self.sync_count / duration
            self.get_logger().info(
                f'Sync rate: {sync_rate:.2f} Hz, Total synced: {self.sync_count}'
            )
        
        self.sync_count = 0
        self.last_sync_time = current_time

def main(args=None):
    rclpy.init(args=args)
    node = DataSynchronizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
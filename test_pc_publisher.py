#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import numpy as np

class TestPublisher(Node):
    def __init__(self):
        super().__init__('test_pc_publisher')
        
        # 使用BEST_EFFORT QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        self.pub = self.create_publisher(PointCloud2, '/velodyne_points', qos)
        self.timer = self.create_timer(0.1, self.publish_test_cloud)
        self.count = 0
        
    def publish_test_cloud(self):
        # 创建简单的测试点云
        msg = PointCloud2()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'velodyne'
        
        # 创建100个随机点
        points = np.random.randn(100, 4).astype(np.float32)
        
        msg.height = 1
        msg.width = 100
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        
        self.pub.publish(msg)
        self.count += 1
        
        if self.count % 10 == 0:
            self.get_logger().info(f'Published {self.count} test clouds')

def main():
    rclpy.init()
    node = TestPublisher()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
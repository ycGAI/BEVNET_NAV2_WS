#!/usr/bin/env python3
"""
BEVNet ROS2集成
'EOF'BEVNet输出
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import numpy as np
import struct
import time

class PointCloudPublisher(Node):
    def __init__(self):
        super().__init__('pointcloud_publisher')
        self.publisher = self.create_publisher(PointCloud2, '/velodyne_points', 10)
        self.timer = self.create_timer(0.1, self.publish_pointcloud)  # 10Hz
        self.frame_count = 0
        
    def publish_pointcloud(self):
        # 生成模拟点云
        num_points = 50000
        points = np.zeros((num_points, 4), dtype=np.float32)
        
        # 环形扫描模式
        angles = np.random.uniform(0, 2*np.pi, num_points)
        distances = np.random.uniform(0.5, 30, num_points)
        heights = np.random.uniform(-2, 0.5, num_points)
        
        points[:, 0] = distances * np.cos(angles)
        points[:, 1] = distances * np.sin(angles)
        points[:, 2] = heights
        points[:, 3] = np.random.uniform(0, 1, num_points)
        
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'velodyne'
        
        msg.height = 1
        msg.width = num_points
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * num_points
        msg.is_dense = True
        
        # 'EOF'
        buffer = []
        for i in range(num_points):
            buffer.append(struct.pack('ffff', points[i,0], points[i,1], points[i,2], points[i,3]))
        msg.data = b''.join(buffer)
        
        self.publisher.publish(msg)
        self.frame_count += 1
        
        if self.frame_count % 10 == 0:
            self.get_logger().info(f'Published frame {self.frame_count}')

def main():
    rclpy.init()
    
    # 创建两个节点
    executor = rclpy.executors.MultiThreadedExecutor()
    
    # 点云发布器
    pc_publisher = PointCloudPublisher()
    executor.add_node(pc_publisher)
    
    print("Starting point cloud publisher...")
    print("In another terminal, run:")
    print("  ros2 run bevnet_nav2_core bevnet_inference_node")
    print("")
    print("To visualize:")
    print("  ros2 topic echo /bevnet/costmap --once")
    print("  ros2 run rviz2 rviz2")
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        pc_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

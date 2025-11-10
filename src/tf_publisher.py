#!/usr/bin/env python3
"""
TF Publisher for BEVNet Navigation System
从poses.txt读取机器人位置并发布TF变换
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
import tf2_ros
import math
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2
import numpy as np
import os


class ImprovedTFPublisher(Node):
    def __init__(self):
        super().__init__('improved_tf_publisher')
        
        # 声明参数
        self.declare_parameter('poses_file', 
            '/workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/sequences/train/poses.txt')
        self.declare_parameter('publish_rate', 30.0)  # 提高发布频率
        
        # 获取参数
        poses_file = self.get_parameter('poses_file').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_tf_broadcaster = StaticTransformBroadcaster(self)
        
        # 加载poses
        self.poses = self.load_poses(poses_file)
        self.current_pose_idx = 0
        
        # 机器人当前位置
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_z = 0.0
        self.robot_qx = 0.0
        self.robot_qy = 0.0
        self.robot_qz = 0.0
        self.robot_qw = 1.0
        
        # 里程计发布器
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        
        # AMCL位姿发布器（模拟定位）
        self.amcl_pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 
            '/amcl_pose', 
            10
        )
        
        # 订阅点云以同步位置更新
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            10
        )
        
        # 定时发布TF
        self.timer = self.create_timer(1.0/publish_rate, self.publish_transforms)
        
        # 发布静态TF
        self.publish_static_transforms()
        
        self.get_logger().info(f'Improved TF Publisher started')
        self.get_logger().info(f'Loaded {len(self.poses)} poses from {poses_file}')
        
        if len(self.poses) > 0:
            self.update_robot_pose(0)
    
    def load_poses(self, poses_file):
        """加载poses.txt文件"""
        poses = []
        
        if not os.path.exists(poses_file):
            self.get_logger().error(f'Poses file not found: {poses_file}')
            return poses
        
        with open(poses_file, 'r') as f:
            lines = f.readlines()
        
        for i, line in enumerate(lines):
            values = [float(v) for v in line.strip().split()]
            
            if len(values) == 12:
                # 构建4x4变换矩阵
                pose_matrix = np.zeros((4, 4))
                pose_matrix[0, 0:4] = values[0:4]
                pose_matrix[1, 0:4] = values[4:8]
                pose_matrix[2, 0:4] = values[8:12]
                pose_matrix[3, 3] = 1.0
                
                # 提取位置
                x = pose_matrix[0, 3]
                y = pose_matrix[1, 3]
                z = pose_matrix[2, 3]
                
                # 从旋转矩阵提取四元数
                quat = self.rotation_matrix_to_quaternion(pose_matrix[:3, :3])
                
                poses.append({
                    'x': x,
                    'y': y,
                    'z': z,
                    'qx': quat[0],
                    'qy': quat[1],
                    'qz': quat[2],
                    'qw': quat[3]
                })
        
        return poses
    
    def rotation_matrix_to_quaternion(self, R):
        """旋转矩阵转四元数"""
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        
        # 归一化
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            return [x/norm, y/norm, z/norm, w/norm]
        else:
            return [0, 0, 0, 1]
    
    def update_robot_pose(self, idx):
        """更新机器人位置"""
        if idx < len(self.poses):
            pose = self.poses[idx]
            self.robot_x = pose['x']
            self.robot_y = pose['y']
            self.robot_z = pose['z']
            self.robot_qx = pose['qx']
            self.robot_qy = pose['qy']
            self.robot_qz = pose['qz']
            self.robot_qw = pose['qw']
            self.current_pose_idx = idx
    
    def pointcloud_callback(self, msg):
        """点云回调，用于同步位置更新"""
        # 逐步更新位置（模拟机器人移动）
        if len(self.poses) > 0:
            self.current_pose_idx = (self.current_pose_idx + 1) % len(self.poses)
            self.update_robot_pose(self.current_pose_idx)
    
    def publish_static_transforms(self):
        """发布静态变换"""
        transforms = []
        
        # map -> odom (静态，假设没有漂移)
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'odom'
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        transforms.append(t)
        
        # base_link -> velodyne (传感器安装位置)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'base_link'
        t2.child_frame_id = 'velodyne'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.5
        t2.transform.rotation.x = 0.0
        t2.transform.rotation.y = 0.0
        t2.transform.rotation.z = 0.0
        t2.transform.rotation.w = 1.0
        transforms.append(t2)
        
        self.static_tf_broadcaster.sendTransform(transforms)
        self.get_logger().info('Published static transforms')
    
    def publish_transforms(self):
        """发布动态变换"""
        current_time = self.get_clock().now().to_msg()
        
        # 发布 odom -> base_link
        odom_to_base = TransformStamped()
        odom_to_base.header.stamp = current_time
        odom_to_base.header.frame_id = 'odom'
        odom_to_base.child_frame_id = 'base_link'
        odom_to_base.transform.translation.x = self.robot_x
        odom_to_base.transform.translation.y = self.robot_y
        odom_to_base.transform.translation.z = self.robot_z
        odom_to_base.transform.rotation.x = self.robot_qx
        odom_to_base.transform.rotation.y = self.robot_qy
        odom_to_base.transform.rotation.z = self.robot_qz
        odom_to_base.transform.rotation.w = self.robot_qw
        
        self.tf_broadcaster.sendTransform(odom_to_base)
        
        # 发布里程计消息
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        
        odom_msg.pose.pose.position.x = self.robot_x
        odom_msg.pose.pose.position.y = self.robot_y
        odom_msg.pose.pose.position.z = self.robot_z
        odom_msg.pose.pose.orientation.x = self.robot_qx
        odom_msg.pose.pose.orientation.y = self.robot_qy
        odom_msg.pose.pose.orientation.z = self.robot_qz
        odom_msg.pose.pose.orientation.w = self.robot_qw
        
        # 设置协方差
        odom_msg.pose.covariance[0] = 0.01  # x
        odom_msg.pose.covariance[7] = 0.01  # y
        odom_msg.pose.covariance[35] = 0.01  # yaw
        
        self.odom_pub.publish(odom_msg)
        
        # 发布AMCL位姿（模拟定位）
        amcl_pose = PoseWithCovarianceStamped()
        amcl_pose.header.stamp = current_time
        amcl_pose.header.frame_id = 'map'
        
        amcl_pose.pose.pose.position.x = self.robot_x
        amcl_pose.pose.pose.position.y = self.robot_y
        amcl_pose.pose.pose.position.z = self.robot_z
        amcl_pose.pose.pose.orientation.x = self.robot_qx
        amcl_pose.pose.pose.orientation.y = self.robot_qy
        amcl_pose.pose.pose.orientation.z = self.robot_qz
        amcl_pose.pose.pose.orientation.w = self.robot_qw
        
        # 设置协方差
        for i in range(36):
            amcl_pose.pose.covariance[i] = 0.0
        amcl_pose.pose.covariance[0] = 0.01  # x
        amcl_pose.pose.covariance[7] = 0.01  # y
        amcl_pose.pose.covariance[35] = 0.01  # yaw
        
        self.amcl_pose_pub.publish(amcl_pose)


def main(args=None):
    rclpy.init(args=args)
    
    node = ImprovedTFPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
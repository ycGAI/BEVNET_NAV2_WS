#!/usr/bin/env python3
"""
简化版 GPS Waypoint Follower
只发布全局路径供 local_planner 使用，不依赖 Nav2
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math


class GPSWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')
        
        # 声明参数
        self.declare_parameter('poses_file', '')
        self.declare_parameter('waypoint_spacing', 5.0)  # 路径点间隔（米）
        self.declare_parameter('skip_initial', 10)        # 跳过初始点数
        self.declare_parameter('loop', False)             # 是否循环路径
        self.declare_parameter('publish_rate', 1.0)       # 发布频率（Hz）
        
        # 获取参数
        poses_file = self.get_parameter('poses_file').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.skip_initial = self.get_parameter('skip_initial').value
        self.loop = self.get_parameter('loop').value
        publish_rate = self.get_parameter('publish_rate').value
        
        if not poses_file:
            self.get_logger().error('No poses file specified!')
            return
            
        # 加载路径点
        self.waypoints = self.load_poses(poses_file)
        if not self.waypoints:
            self.get_logger().error('Failed to load waypoints!')
            return
            
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        # 发布器 - 使用 /plan 话题（Nav2 标准话题名）
        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        self.path_pub = self.create_publisher(Path, '/plan', qos)
        
        # 也发布到 /gps_path 用于可视化
        self.viz_path_pub = self.create_publisher(Path, '/gps_path', 10)
        
        # 定时发布路径
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_path)
        
        # 立即发布一次
        self.publish_path()
        
        self.get_logger().info('='*50)
        self.get_logger().info('GPS Waypoint Follower Started (Simplified)')
        self.get_logger().info(f'  Poses file: {poses_file}')
        self.get_logger().info(f'  Waypoints: {len(self.waypoints)}')
        self.get_logger().info(f'  Spacing: {self.waypoint_spacing}m')
        self.get_logger().info(f'  Publishing to: /plan')
        self.get_logger().info('='*50)
    
    def load_poses(self, poses_file):
        """加载并处理 poses.txt 文件"""
        if not os.path.exists(poses_file):
            self.get_logger().error(f'Poses file not found: {poses_file}')
            return []
        
        waypoints = []
        last_waypoint = None
        
        with open(poses_file, 'r') as f:
            lines = f.readlines()
            
        self.get_logger().info(f'Processing {len(lines)} poses...')
        
        # 跳过初始的一些点
        start_idx = min(self.skip_initial, len(lines) - 1)
        
        for i, line in enumerate(lines[start_idx:], start=start_idx):
            values = [float(v) for v in line.strip().split()]
            
            if len(values) == 12:
                # 构建 4x4 变换矩阵
                pose_matrix = np.zeros((4, 4))
                pose_matrix[0, 0:4] = values[0:4]
                pose_matrix[1, 0:4] = values[4:8]
                pose_matrix[2, 0:4] = values[8:12]
                pose_matrix[3, 3] = 1.0
                
                # 提取位置
                x = pose_matrix[0, 3]
                y = pose_matrix[1, 3]
                
                # 检查间距（避免路径点过密）
                if last_waypoint is None:
                    # 第一个点
                    waypoint = self.matrix_to_pose(pose_matrix)
                    waypoints.append(waypoint)
                    last_waypoint = (x, y)
                else:
                    # 计算与上一个路径点的距离
                    dist = math.sqrt((x - last_waypoint[0])**2 + 
                                     (y - last_waypoint[1])**2)
                    
                    # 只添加间隔足够的点
                    if dist >= self.waypoint_spacing:
                        waypoint = self.matrix_to_pose(pose_matrix)
                        waypoints.append(waypoint)
                        last_waypoint = (x, y)
        
        # 如果需要循环，添加回到起点的路径
        if self.loop and len(waypoints) > 1:
            waypoints.append(waypoints[0])
        
        self.get_logger().info(f'Filtered to {len(waypoints)} waypoints '
                               f'with {self.waypoint_spacing}m spacing')
        
        return waypoints
    
    def matrix_to_pose(self, matrix):
        """将 4x4 变换矩阵转换为 PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # 位置
        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = 0.0  # 2D 导航
        
        # 从旋转矩阵计算四元数
        R = matrix[:3, :3]
        quat = self.rotation_matrix_to_quaternion(R)
        
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]
        
        return pose
    
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
            return [0.0, 0.0, 0.0, 1.0]
    
    def publish_path(self):
        """发布全局路径"""
        if not self.waypoints:
            return
        
        # 更新时间戳
        current_time = self.get_clock().now().to_msg()
        
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = current_time
        
        # 更新每个 waypoint 的时间戳
        for wp in self.waypoints:
            wp.header.stamp = current_time
        
        path_msg.poses = self.waypoints
        
        # 发布到 /plan（local_planner 订阅）
        self.path_pub.publish(path_msg)
        
        # 发布到 /gps_path（可视化）
        self.viz_path_pub.publish(path_msg)

        self.get_logger().info(f'Published path with {len(self.waypoints)} waypoints')


def main(args=None):
    rclpy.init(args=args)
    
    node = GPSWaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
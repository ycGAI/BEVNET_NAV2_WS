#!/usr/bin/env python3
"""
GPS Waypoint Follower for KITTI poses
使用poses.txt中的轨迹点作为全局路径，配合BEVNet的局部规划
"""

import os
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from rclpy.action import ActionClient
import tf2_ros
from tf2_ros import TransformException
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
import math


class GPSWaypointFollower(Node):
    def __init__(self):
        super().__init__('gps_waypoint_follower')
        
        # 声明参数
        self.declare_parameter('poses_file', '')
        self.declare_parameter('waypoint_spacing', 5.0)  # 路径点间隔（米）
        self.declare_parameter('goal_tolerance', 2.0)    # 到达目标的容差（米）
        self.declare_parameter('loop', False)             # 是否循环
        self.declare_parameter('skip_initial', 10)        # 跳过初始点数（避免起点附近密集点）
        
        # 获取参数
        poses_file = self.get_parameter('poses_file').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.loop = self.get_parameter('loop').value
        self.skip_initial = self.get_parameter('skip_initial').value
        
        if not poses_file:
            self.get_logger().error('No poses file specified!')
            return
            
        # 加载路径点
        self.waypoints = self.load_poses(poses_file)
        if not self.waypoints:
            self.get_logger().error('Failed to load waypoints!')
            return
            
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        
        # 当前目标索引
        self.current_waypoint_idx = 0
        self.robot_pose = None
        
        # Nav2动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # TF监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # 发布器
        qos = QoSProfile(
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            reliability=ReliabilityPolicy.RELIABLE,
            depth=10
        )
        
        # 发布完整路径用于可视化
        self.path_pub = self.create_publisher(Path, '/gps_path', qos)
        
        # 发布当前目标
        self.goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)
        
        # 订阅机器人位置
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_pose_callback,
            10
        )
        
        # 定时器
        # 定时器
        self.timer = self.create_timer(1.0, self.update_navigation)
        # 添加路径发布定时器 - 持续发布供可视化
        self.path_publish_timer = self.create_timer(1.0, self.publish_path)

        # 发布初始路径
        self.publish_path()

        # 等待Nav2就绪（添加超时避免永久阻塞）
        self.get_logger().info('Waiting for Nav2 action server...')
        if not self.nav_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().warn('Nav2 not ready after 10s, continuing anyway...')
        else:
            self.get_logger().info('Nav2 ready! Starting navigation...')
            # 开始导航
            self.navigate_to_next_waypoint()
        self.get_logger().info('Nav2 ready! Starting navigation...')
        
        # 开始导航
        self.navigate_to_next_waypoint()
    
    def load_poses(self, poses_file):
        """加载并处理poses.txt文件"""
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
                
                # 检查间距（避免路径点过密）
                if last_waypoint is None:
                    # 第一个点
                    waypoint = self.matrix_to_pose(pose_matrix, i)
                    waypoints.append(waypoint)
                    last_waypoint = (x, y)
                else:
                    # 计算与上一个路径点的距离
                    dist = math.sqrt((x - last_waypoint[0])**2 + 
                                   (y - last_waypoint[1])**2)
                    
                    # 只添加间隔足够的点
                    if dist >= self.waypoint_spacing:
                        waypoint = self.matrix_to_pose(pose_matrix, i)
                        waypoints.append(waypoint)
                        last_waypoint = (x, y)
        
        self.get_logger().info(f'Filtered to {len(waypoints)} waypoints '
                               f'with {self.waypoint_spacing}m spacing')
        
        return waypoints
    
    def matrix_to_pose(self, matrix, index):
        """将4x4变换矩阵转换为PoseStamped"""
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        # 位置
        pose.pose.position.x = matrix[0, 3]
        pose.pose.position.y = matrix[1, 3]
        pose.pose.position.z = 0.0  # 2D导航，z设为0
        
        # 从旋转矩阵计算四元数
        R = matrix[:3, :3]
        
        # 计算四元数
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
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
        
        # 归一化四元数
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            pose.pose.orientation.w = w / norm
            pose.pose.orientation.x = x / norm
            pose.pose.orientation.y = y / norm
            pose.pose.orientation.z = z / norm
        else:
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
        
        return pose
    
    def publish_path(self):
        """发布完整路径用于可视化"""
        path_msg = Path()
        path_msg.header.frame_id = 'map'
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.poses = self.waypoints
        
        self.path_pub.publish(path_msg)
        self.get_logger().info('Published GPS path for visualization')
    
    def get_robot_pose(self):
        """获取机器人当前位置"""
        try:
            # 尝试从TF获取机器人位置
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_link',  # 或 'velodyne'
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0)
            )
            
            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except TransformException as ex:
            self.get_logger().debug(f'Could not get robot pose: {ex}')
            return self.robot_pose  # 使用上次的位置
    
    def amcl_pose_callback(self, msg):
        """AMCL位姿回调"""
        self.robot_pose = PoseStamped()
        self.robot_pose.header = msg.header
        self.robot_pose.pose = msg.pose.pose
    
    def find_nearest_waypoint(self):
        """找到最近的未访问路径点"""
        robot_pose = self.get_robot_pose()
        if not robot_pose:
            return self.current_waypoint_idx
        
        min_dist = float('inf')
        nearest_idx = self.current_waypoint_idx
        
        # 从当前索引开始搜索（避免回头）
        for i in range(self.current_waypoint_idx, len(self.waypoints)):
            wp = self.waypoints[i]
            dist = self.calculate_distance(robot_pose, wp)
            
            if dist < min_dist:
                min_dist = dist
                nearest_idx = i
        
        return nearest_idx
    
    def calculate_distance(self, pose1, pose2):
        """计算两个位姿之间的距离"""
        dx = pose1.pose.position.x - pose2.pose.position.x
        dy = pose1.pose.position.y - pose2.pose.position.y
        return math.sqrt(dx*dx + dy*dy)
    
    def navigate_to_next_waypoint(self):
        """导航到下一个路径点"""
        if self.current_waypoint_idx >= len(self.waypoints):
            if self.loop:
                self.current_waypoint_idx = 0
                self.get_logger().info('Looping back to start')
            else:
                self.get_logger().info('Navigation complete!')
                return
        
        # 找最近的路径点
        self.current_waypoint_idx = self.find_nearest_waypoint()
        
        waypoint = self.waypoints[self.current_waypoint_idx]
        self.get_logger().info(f'Navigating to waypoint {self.current_waypoint_idx + 1}/{len(self.waypoints)} '
                              f'at ({waypoint.pose.position.x:.2f}, {waypoint.pose.position.y:.2f})')
        
        # 发布当前目标
        self.goal_pub.publish(waypoint)
        
        # 发送导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = waypoint
        
        self.nav_future = self.nav_client.send_goal_async(goal_msg)
        self.nav_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        """导航目标响应回调"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted!')
        
        # 获取结果
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)
    
    def goal_result_callback(self, future):
        """导航结果回调"""
        result = future.result().result
        status = future.result().status
        
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx + 1}!')
            self.current_waypoint_idx += 1
            
            # 导航到下一个点
            self.navigate_to_next_waypoint()
        else:
            self.get_logger().warn(f'Navigation failed with status: {status}')
            # 可以选择跳过这个点或重试
            self.current_waypoint_idx += 1
            self.navigate_to_next_waypoint()
    
    def update_navigation(self):
        """定期检查导航状态"""
        # 检查是否需要更新路径点
        robot_pose = self.get_robot_pose()
        
        if robot_pose and self.current_waypoint_idx < len(self.waypoints):
            # 检查是否接近当前目标
            current_goal = self.waypoints[self.current_waypoint_idx]
            dist = self.calculate_distance(robot_pose, current_goal)
            
            if dist < self.goal_tolerance:
                self.get_logger().info(f'Close enough to waypoint {self.current_waypoint_idx + 1} '
                                      f'(distance: {dist:.2f}m)')
                self.current_waypoint_idx += 1
                self.navigate_to_next_waypoint()


def main():
    rclpy.init()
    
    node = GPSWaypointFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
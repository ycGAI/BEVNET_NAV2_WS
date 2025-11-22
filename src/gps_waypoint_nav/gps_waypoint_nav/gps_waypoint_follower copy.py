#!/usr/bin/env python3
"""
容错GPS Waypoint Follower for KITTI数据集回放
- 使用GPS航点进行全局路径规划
- 使用BEVNet进行局部避障
- 即使导航失败也继续规划下一个点
- 可视化全局和局部路径
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose, ComputePathToPose
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import time


class TolerantGPSFollower(Node):
    def __init__(self):
        super().__init__('tolerant_gps_follower')
        
        # 参数
        self.declare_parameter('poses_file', '/workspace/data/kitti/poses.txt')
        self.declare_parameter('waypoint_spacing', 5.0)  # 降低间隔
        self.declare_parameter('continue_on_failure', True)  # 失败也继续
        self.declare_parameter('timeout_seconds', 3.0)  # 每个点的超时时间
        
        poses_file = self.get_parameter('poses_file').value
        self.waypoint_spacing = self.get_parameter('waypoint_spacing').value
        self.continue_on_failure = self.get_parameter('continue_on_failure').value
        self.timeout = self.get_parameter('timeout_seconds').value
        
        # 动作客户端
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.planner_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        # 发布器
        self.global_path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.local_path_pub = self.create_publisher(Path, '/local_plan', 10)
        self.waypoints_pub = self.create_publisher(MarkerArray, '/gps_waypoints', 10)
        self.current_goal_pub = self.create_publisher(PoseStamped, '/current_goal', 10)
        
        # 订阅
        self.robot_pose = None
        self.pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            '/amcl_pose',
            self.amcl_callback,
            10
        )
        
        # 订阅Nav2的路径（用于可视化）
        self.plan_sub = self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )
        
        # 加载航点
        self.waypoints = self.load_waypoints(poses_file)
        if not self.waypoints:
            self.get_logger().error('No waypoints loaded!')
            return
            
        self.current_waypoint_idx = 0
        self.navigation_active = False
        
        self.get_logger().info(f'Loaded {len(self.waypoints)} waypoints')
        self.get_logger().info('Tolerant GPS Follower started!')
        self.get_logger().info('Will continue planning even if navigation fails')
        
        # 发布航点标记
        self.publish_waypoint_markers()
        
        # 等待系统初始化
        self.create_timer(3.0, self.start_navigation)
        
        # 定时器：持续尝试导航
        self.nav_timer = self.create_timer(self.timeout, self.navigation_loop)
        
        # 定时器：发布可视化
        self.vis_timer = self.create_timer(0.5, self.publish_visualization)
    
    def load_waypoints(self, poses_file):
        """从文件加载GPS航点"""
        waypoints = []
        
        try:
            # 如果文件不存在，使用默认航点
            try:
                with open(poses_file, 'r') as f:
                    lines = f.readlines()
                
                all_poses = []
                for line in lines[:500]:  # 限制数量
                    values = [float(v) for v in line.strip().split()]
                    if len(values) >= 12:
                        # KITTI格式: 3x4变换矩阵
                        x = values[3]
                        y = values[7]
                        all_poses.append((x, y))
            except:
                # 使用默认航点
                self.get_logger().warn('Using default waypoints')
                all_poses = [(i*5, 0) for i in range(20)]  # 直线航点
            
            # 按间隔采样
            if all_poses:
                last_x, last_y = all_poses[0]
                waypoints.append(all_poses[0])
                
                for x, y in all_poses[1:]:
                    dist = np.sqrt((x-last_x)**2 + (y-last_y)**2)
                    if dist >= self.waypoint_spacing:
                        waypoints.append((x, y))
                        last_x, last_y = x, y
            
            self.get_logger().info(f'Sampled {len(waypoints)} waypoints')
            
        except Exception as e:
            self.get_logger().error(f'Error loading waypoints: {e}')
            # 使用备用航点
            waypoints = [(i*5, 0) for i in range(10)]
        
        return waypoints
    
    def amcl_callback(self, msg):
        """更新机器人位置"""
        self.robot_pose = msg.pose.pose
    
    def plan_callback(self, msg):
        """接收Nav2生成的路径用于可视化"""
        # 转发到local_plan话题
        self.local_path_pub.publish(msg)
    
    def start_navigation(self):
        """开始导航"""
        self.get_logger().info('Starting navigation...')
        self.navigation_active = True
        self.send_next_goal()
    
    def navigation_loop(self):
        """导航主循环 - 容错版本"""
        if not self.navigation_active or not self.waypoints:
            return
        
        # 无论成功失败，持续发送下一个目标
        if self.continue_on_failure:
            self.current_waypoint_idx = (self.current_waypoint_idx + 1) % len(self.waypoints)
            self.send_next_goal()
    
    def send_next_goal(self):
        """发送下一个导航目标"""
        if self.current_waypoint_idx >= len(self.waypoints):
            self.current_waypoint_idx = 0  # 循环
        
        wp = self.waypoints[self.current_waypoint_idx]
        
        # 创建导航目标
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp[0]
        goal_msg.pose.pose.position.y = wp[1]
        goal_msg.pose.pose.position.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0
        
        self.get_logger().info(
            f'Planning to waypoint {self.current_waypoint_idx+1}/{len(self.waypoints)} '
            f'at ({wp[0]:.2f}, {wp[1]:.2f})'
        )
        
        # 发送导航目标（不等待结果）
        if self.nav_client.wait_for_server(timeout_sec=0.5):
            self.nav_client.send_goal_async(goal_msg)
        
        # 同时请求路径规划（用于可视化）
        if self.planner_client.wait_for_server(timeout_sec=0.5):
            plan_goal = ComputePathToPose.Goal()
            plan_goal.pose = goal_msg.pose
            plan_goal.planner_id = 'GridBased'
            self.planner_client.send_goal_async(plan_goal)
        
        # 发布当前目标
        current_goal = PoseStamped()
        current_goal.header = goal_msg.pose.header
        current_goal.pose = goal_msg.pose.pose
        self.current_goal_pub.publish(current_goal)
    
    def publish_waypoint_markers(self):
        """发布航点标记用于RViz可视化"""
        marker_array = MarkerArray()
        
        for i, wp in enumerate(self.waypoints):
            marker = Marker()
            marker.header.frame_id = 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'gps_waypoints'
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            marker.pose.position.x = wp[0]
            marker.pose.position.y = wp[1]
            marker.pose.position.z = 0.5
            marker.pose.orientation.w = 1.0
            
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            
            # 当前目标用红色，其他用绿色
            if i == self.current_waypoint_idx:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            marker.color.a = 0.8
            
            marker_array.markers.append(marker)
        
        self.waypoints_pub.publish(marker_array)
    
    def publish_visualization(self):
        """发布可视化信息"""
        # 发布GPS航点路径
        gps_path = Path()
        gps_path.header.frame_id = 'map'
        gps_path.header.stamp = self.get_clock().now().to_msg()
        
        for wp in self.waypoints:
            pose = PoseStamped()
            pose.header = gps_path.header
            pose.pose.position.x = wp[0]
            pose.pose.position.y = wp[1]
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            gps_path.poses.append(pose)
        
        self.global_path_pub.publish(gps_path)
        
        # 更新航点标记
        self.publish_waypoint_markers()
    
    def feedback_callback(self, feedback_msg):
        """导航反馈回调"""
        # 可以在这里处理导航反馈
        pass


def main():
    rclpy.init()
    
    node = TolerantGPSFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
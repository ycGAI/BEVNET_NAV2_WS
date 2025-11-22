#!/usr/bin/env python3
"""
简单的路径跟踪器，绕过Nav2 Controller的问题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener
import math


class SimplePathFollower(Node):
    def __init__(self):
        super().__init__('simple_path_follower')
        
        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 订阅路径
        self.path_sub = self.create_subscription(
            Path,
            '/plan',
            self.path_callback,
            10
        )
        
        # 发布速度命令
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 控制定时器 (10Hz)
        self.timer = self.create_timer(0.1, self.control_loop)
        
        self.current_path = None
        self.current_waypoint_idx = 0
        
        self.get_logger().info('Simple Path Follower started!')
        self.get_logger().info('Waiting for path on /plan topic...')
    
    def path_callback(self, msg):
        """接收新路径"""
        if msg.poses:
            self.current_path = msg
            self.current_waypoint_idx = 0
            self.get_logger().info(f'Received path with {len(msg.poses)} waypoints')
    
    def get_robot_pose(self):
        """获取机器人当前位置"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', 
                rclpy.time.Time()
            )
            return transform.transform
        except:
            return None
    
    def control_loop(self):
        """简单的控制逻辑"""
        if not self.current_path or not self.current_path.poses:
            return
        
        # 获取机器人位置
        robot_transform = self.get_robot_pose()
        if not robot_transform:
            self.get_logger().warn('Cannot get robot pose', throttle_duration_sec=5.0)
            return
        
        robot_x = robot_transform.translation.x
        robot_y = robot_transform.translation.y
        
        # 获取机器人朝向
        quat = robot_transform.rotation
        robot_yaw = math.atan2(2*(quat.w*quat.z + quat.x*quat.y),
                               1 - 2*(quat.y*quat.y + quat.z*quat.z))
        
        # 获取当前目标点
        if self.current_waypoint_idx >= len(self.current_path.poses):
            # 到达终点
            cmd = Twist()
            self.cmd_pub.publish(cmd)
            self.get_logger().info('Reached goal!', throttle_duration_sec=1.0)
            return
        
        target = self.current_path.poses[self.current_waypoint_idx].pose.position
        
        # 计算到目标的距离和角度
        dx = target.x - robot_x
        dy = target.y - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        target_angle = math.atan2(dy, dx)
        
        # 计算角度差
        angle_diff = target_angle - robot_yaw
        # 归一化到[-pi, pi]
        while angle_diff > math.pi:
            angle_diff -= 2*math.pi
        while angle_diff < -math.pi:
            angle_diff += 2*math.pi
        
        cmd = Twist()
        
        if distance < 0.5:  # 到达当前航点（0.5米容忍度）
            self.current_waypoint_idx += 1
            self.get_logger().info(f'Reached waypoint {self.current_waypoint_idx}/{len(self.current_path.poses)}')
        else:
            # 简单的P控制器
            if abs(angle_diff) > 0.2:  # 需要旋转（0.2弧度 ≈ 11度）
                cmd.angular.z = 0.5 * angle_diff  # P控制
                cmd.angular.z = max(-1.0, min(1.0, cmd.angular.z))  # 限制
                cmd.linear.x = 0.1  # 慢速前进
            else:  # 朝向正确，可以前进
                cmd.linear.x = min(0.5, 0.3 * distance)  # 根据距离调整速度
                cmd.angular.z = 0.1 * angle_diff  # 小幅修正
        
        self.cmd_pub.publish(cmd)
        
        # 调试输出
        self.get_logger().info(
            f'Robot: ({robot_x:.2f},{robot_y:.2f}), Target: ({target.x:.2f},{target.y:.2f}), '
            f'Dist: {distance:.2f}m, Angle: {math.degrees(angle_diff):.1f}°, '
            f'Cmd: v={cmd.linear.x:.2f}, w={cmd.angular.z:.2f}',
            throttle_duration_sec=0.5
        )


def main():
    rclpy.init()
    node = SimplePathFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停止机器人
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
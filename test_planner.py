#!/usr/bin/env python3
"""直接测试Planner服务"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose
from geometry_msgs.msg import PoseStamped


class PlannerTest(Node):
    def __init__(self):
        super().__init__('planner_test')
        
        # Action客户端
        self.client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
        self.get_logger().info('Waiting for planner server...')
        self.client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info('Planner server ready!')
        
        # 测试路径规划
        self.test_planning()
    
    def test_planning(self):
        # 创建起点
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.header.stamp = self.get_clock().now().to_msg()
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.orientation.w = 1.0
        
        # 创建终点
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = 5.0
        goal.pose.position.y = 0.0
        goal.pose.orientation.w = 1.0
        
        # 创建规划请求
        planning_goal = ComputePathToPose.Goal()
        planning_goal.pose = goal  # ROS2 Foxy中是'pose'而不是'goal'
        planning_goal.planner_id = 'GridBased'
        
        self.get_logger().info('Sending planning request to (5,0)...')
        
        # 发送请求
        future = self.client.send_goal_async(planning_goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            goal_handle = future.result()
            if goal_handle.accepted:
                self.get_logger().info('Planning goal accepted!')
                
                # 等待结果
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(self, result_future, timeout_sec=5.0)
                
                if result_future.result():
                    result = result_future.result().result
                    if result.path and result.path.poses:
                        self.get_logger().info(f'✅ Path generated with {len(result.path.poses)} points!')
                        self.get_logger().info(f'First point: ({result.path.poses[0].pose.position.x:.2f}, {result.path.poses[0].pose.position.y:.2f})')
                        self.get_logger().info(f'Last point: ({result.path.poses[-1].pose.position.x:.2f}, {result.path.poses[-1].pose.position.y:.2f})')
                    else:
                        self.get_logger().error('❌ Path is empty!')
                else:
                    self.get_logger().error('❌ Failed to get planning result!')
            else:
                self.get_logger().error('❌ Planning goal rejected!')
        else:
            self.get_logger().error('❌ Failed to send planning goal!')


def main():
    rclpy.init()
    node = PlannerTest()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
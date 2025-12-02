import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist, PointStamped
from nav_msgs.msg import Path, OccupancyGrid, Odometry
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
import math
import numpy as np

from .dwa_planner import DWAPlanner

class LocalPlannerNode(Node):
    def __init__(self):
        super().__init__('local_planner')
        
        # 参数
        self.declare_parameter('goal_tolerance', 2.0)
        self.declare_parameter('control_frequency', 10.0)
        
        self.goal_tolerance = self.get_parameter('goal_tolerance').value
        self.control_frequency = self.get_parameter('control_frequency').value
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 规划器
        self.planner = DWAPlanner()
        
        # 状态变量
        self.current_pose = None
        self.global_plan = None
        self.costmap = None
        self.goal_pose = None
        self.robot_frame = 'base_link'
        self.global_frame = 'map'
        
        # 发布者
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.local_plan_pub = self.create_publisher(Path, '/local_plan', 10)
        
        # 订阅者
        # self.global_plan_sub = self.create_subscription(
        #     Path, 'plan', self.global_plan_callback, 10)
        self.global_plan_sub = self.create_subscription(
            Path, '/plan', self.global_plan_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/bevnet/costmap', self.costmap_callback, 10)
        
        # 定时器
        self.timer = self.create_timer(
            1.0/self.control_frequency, self.control_loop)
        
        self.get_logger().info('Local planner node started')
    
    def global_plan_callback(self, msg):
        """全局路径回调"""
        if len(msg.poses) > 0:
            self.global_plan = msg
            # 设置目标点
            goal_pose = msg.poses[-1]
            self.goal_pose = [
                goal_pose.pose.position.x,
                goal_pose.pose.position.y
            ]
            self.get_logger().info(f'Received global plan with {len(msg.poses)} points')
    
    # def odom_callback(self, msg):
    #     """里程计回调"""
    #     try:
    #         # 获取从odom到base_link的变换
    #         transform = self.tf_buffer.lookup_transform(
    #             self.global_frame, self.robot_frame, rclpy.time.Time())
            
    #         # 创建点并变换
    #         point = PointStamped()
    #         point.header.frame_id = self.robot_frame
    #         point.point.x = 0.0
    #         point.point.y = 0.0
    #         point.point.z = 0.0
            
    #         transformed_point = do_transform_point(point, transform)
            
    #         # 获取朝向
    #         orientation = msg.pose.pose.orientation
    #         q0 = orientation.x
    #         q1 = orientation.y
    #         q2 = orientation.z
    #         q3 = orientation.w
    #         theta = math.atan2(2*(q3*q2 + q0*q1), 1-2*(q1*q1 + q2*q2))
            
    #         self.current_pose = [
    #             transformed_point.point.x,
    #             transformed_point.point.y,
    #             theta
    #         ]
            
    #     except Exception as e:
    #         self.get_logger().warn(f'Transform error: {str(e)}')
    def odom_callback(self, msg):
        """里程计回调 - 简化版，直接使用odom数据"""
        # 直接从 odom 消息获取位置（因为 map ≈ odom）
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation
        
        # 计算 yaw 角
        q0 = orientation.x
        q1 = orientation.y
        q2 = orientation.z
        q3 = orientation.w
        theta = math.atan2(2*(q3*q2 + q0*q1), 1-2*(q1*q1 + q2*q2))
        
        self.current_pose = [
            position.x,
            position.y,
            theta
        ]
    
    def costmap_callback(self, msg):
        """代价地图回调"""
        self.costmap = msg
    
    def get_local_goal(self, lookahead_distance=1.0):
        """从全局路径中获取局部目标点"""
        if self.global_plan is None or self.current_pose is None:
            return None
        
        # 找到距离机器人最近的点
        min_distance = float('inf')
        closest_index = 0
        
        for i, pose in enumerate(self.global_plan.poses):
            dx = pose.pose.position.x - self.current_pose[0]
            dy = pose.pose.position.y - self.current_pose[1]
            distance = math.sqrt(dx*dx + dy*dy)
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # 向前看一定距离
        lookahead_index = closest_index
        accumulated_distance = 0.0
        
        for i in range(closest_index, len(self.global_plan.poses)-1):
            dx = self.global_plan.poses[i+1].pose.position.x - self.global_plan.poses[i].pose.position.x
            dy = self.global_plan.poses[i+1].pose.position.y - self.global_plan.poses[i].pose.position.y
            segment_length = math.sqrt(dx*dx + dy*dy)
            accumulated_distance += segment_length
            
            if accumulated_distance >= lookahead_distance:
                lookahead_index = i
                break
            lookahead_index = i
        
        if lookahead_index < len(self.global_plan.poses):
            goal_pose = self.global_plan.poses[lookahead_index]
            return [goal_pose.pose.position.x, goal_pose.pose.position.y]
        
        return self.goal_pose
    
    def is_goal_reached(self):
        """检查是否到达目标"""
        if self.current_pose is None or self.goal_pose is None:
            return False
        
        dx = self.goal_pose[0] - self.current_pose[0]
        dy = self.goal_pose[1] - self.current_pose[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        return distance <= self.goal_tolerance
    
    def publish_local_plan(self, trajectory):
        """发布局部路径"""
        if trajectory is None or len(trajectory) == 0:
            return
            
        path_msg = Path()
        path_msg.header.frame_id = self.global_frame
        path_msg.header.stamp = self.get_clock().now().to_msg()
        
        for point in trajectory:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.position.z = -1.5
            path_msg.poses.append(pose)
        
        self.local_plan_pub.publish(path_msg)
    
    def control_loop(self):
        """控制循环"""
        # 检查数据是否就绪
        if self.current_pose is None:
            self.get_logger().warn('No current_pose')
            return
        
        if self.global_plan is None:
            self.get_logger().warn('No global_plan')
            return
            
        if self.goal_pose is None:
            self.get_logger().warn('No goal_pose')
            return
        
        self.get_logger().info(f'Robot at: ({self.current_pose[0]:.2f}, {self.current_pose[1]:.2f})')
        
        if self.is_goal_reached():
            cmd_vel = Twist()
            self.cmd_vel_pub.publish(cmd_vel)
            self.get_logger().info('Goal reached!')
            return
        
        # 获取局部目标点
        local_goal = self.get_local_goal()
        if local_goal is None:
            self.get_logger().warn('No local_goal')
            return
        
        self.get_logger().info(f'Local goal: ({local_goal[0]:.2f}, {local_goal[1]:.2f})')
        
        # 执行DWA规划
        velocity, trajectory = self.planner.plan(
            self.current_pose, local_goal, self.global_plan, self.costmap
        )
        
        traj_len = len(trajectory) if trajectory is not None and len(trajectory) > 0 else 0
        self.get_logger().info(f'DWA result: v={velocity[0]:.2f}, w={velocity[1]:.2f}, traj_len={traj_len}')
        
        # 发布速度命令
        cmd_vel = Twist()
        cmd_vel.linear.x = velocity[0]
        cmd_vel.angular.z = velocity[1]
        self.cmd_vel_pub.publish(cmd_vel)
        
        # 发布局部路径
        self.publish_local_plan(trajectory)

def main(args=None):
    rclpy.init(args=args)
    node = LocalPlannerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
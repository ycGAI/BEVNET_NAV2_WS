import math
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist, Point
from nav_msgs.msg import Path, OccupancyGrid
from tf2_ros import TransformException
from tf2_geometry_msgs import do_transform_point
import rclpy
from rclpy.node import Node

class DWAPlanner:
    def __init__(self):
        # DWA参数
        self.max_vel_x = 0.5      # 最大线速度
        self.min_vel_x = -0.2     # 最小线速度（后退）
        self.max_vel_theta = 1.0  # 最大角速度
        
        self.vel_resolution = 0.05    # 速度分辨率
        self.vel_theta_resolution = 0.1
        
        self.predict_time = 1.5      # 预测时间
        self.dt = 0.2               # 时间步长
        
        # 代价函数权重
        self.heading_weight = 0.3
        self.velocity_weight = 0.3
        self.obstacle_weight = 0.4
        
        # 障碍物参数
        self.obstacle_radius = 0.3
        
    def motion_model(self, x, u, dt):
        """运动模型"""
        x[0] += u[0] * math.cos(x[2]) * dt  # x
        x[1] += u[0] * math.sin(x[2]) * dt  # y
        x[2] += u[1] * dt                   # theta
        return x
    
    def generate_trajectory(self, x, v, w, predict_time, dt):
        """生成轨迹"""
        trajectory = [x.copy()]
        time = 0
        while time <= predict_time:
            x = self.motion_model(x, [v, w], dt)
            trajectory.append(x.copy())
            time += dt
        return np.array(trajectory)
    
    def calculate_heading_cost(self, trajectory, goal_pose):
        """计算朝向代价"""
        if len(trajectory) == 0:
            return float('inf')
            
        dx = goal_pose[0] - trajectory[-1][0]
        dy = goal_pose[1] - trajectory[-1][1]
        goal_angle = math.atan2(dy, dx)
        
        # 轨迹终点朝向与目标点方向的差异
        angle_diff = abs(goal_angle - trajectory[-1][2])
        cost = abs(math.atan2(math.sin(angle_diff), math.cos(angle_diff)))
        
        return cost
    
    def calculate_velocity_cost(self, v):
        """计算速度代价"""
        return self.max_vel_x - abs(v)
    
    def calculate_obstacle_cost(self, trajectory, obstacles):
        """计算障碍物代价"""
        if len(obstacles) == 0:
            return 0.0
            
        min_distance = float('inf')
        for point in trajectory:
            for obstacle in obstacles:
                distance = math.sqrt((point[0]-obstacle[0])**2 + (point[1]-obstacle[1])**2)
                if distance < min_distance:
                    min_distance = distance
        
        if min_distance <= self.obstacle_radius:
            return float('inf')
            
        return 1.0 / min_distance
    
    def get_obstacles_from_costmap(self, costmap, robot_pose):
        """从代价地图获取障碍物"""
        obstacles = []
        if costmap is None:
            return obstacles
            
        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        
        for i in range(width):
            for j in range(height):
                index = j * width + i
                if costmap.data[index] > 50:  # 认为是障碍物
                    # 转换为世界坐标
                    ox = origin_x + (i + 0.5) * resolution
                    oy = origin_y + (j + 0.5) * resolution
                    obstacles.append([ox, oy])
                    
        return obstacles
    
    def plan(self, current_pose, goal_pose, global_plan, costmap=None):
        """
        执行DWA规划
        current_pose: [x, y, theta]
        goal_pose: [x, y]
        global_plan: 全局路径
        costmap: 代价地图
        """
        # 获取障碍物信息
        obstacles = self.get_obstacles_from_costmap(costmap, current_pose)
        
        # 速度采样空间
        min_vel_x = max(self.min_vel_x, current_pose[0] - 0.1)
        max_vel_x = min(self.max_vel_x, current_pose[0] + 0.1)
        
        best_trajectory = None
        min_cost = float('inf')
        best_velocity = [0.0, 0.0]
        
        # 速度采样
        for v in np.arange(min_vel_x, max_vel_x + self.vel_resolution, self.vel_resolution):
            for w in np.arange(-self.max_vel_theta, self.max_vel_theta + self.vel_theta_resolution, self.vel_theta_resolution):
                if abs(v) < 0.01 and abs(w) < 0.01:
                    continue
                    
                # 生成轨迹
                trajectory = self.generate_trajectory(
                    current_pose.copy(), v, w, self.predict_time, self.dt
                )
                
                # 计算各项代价
                heading_cost = self.calculate_heading_cost(trajectory, goal_pose)
                velocity_cost = self.calculate_velocity_cost(v)
                obstacle_cost = self.calculate_obstacle_cost(trajectory, obstacles)
                
                # 总代价
                total_cost = (self.heading_weight * heading_cost +
                            self.velocity_weight * velocity_cost +
                            self.obstacle_weight * obstacle_cost)
                
                if total_cost < min_cost and obstacle_cost < float('inf'):
                    min_cost = total_cost
                    best_trajectory = trajectory
                    best_velocity = [v, w]
        
        return best_velocity, best_trajectory
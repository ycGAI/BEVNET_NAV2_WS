import math
import numpy as np
from nav_msgs.msg import Path, OccupancyGrid

class DWAPlanner:
    def __init__(self):
        # DWA参数
        self.max_vel_x = 0.5
        self.min_vel_x = 0.0
        self.max_vel_theta = 0.3   # 限制最大转向
        
        self.vel_resolution = 0.1
        self.vel_theta_resolution = 0.05
        
        self.predict_time = 2.0
        self.dt = 0.2
        
        # 代价函数权重
        self.path_direction_weight = 0.5
        self.velocity_weight = 0.1
        self.obstacle_weight = 0.2
        self.smoothness_weight = 0.2
        
        # 障碍物参数
        self.obstacle_radius = 0.5
        self.obstacle_check_radius = 5.0
        
        # 速度平滑
        self.last_v = 0.0
        self.last_w = 0.0
        self.velocity_filter_alpha = 0.4
        
    def motion_model(self, x, u, dt):
        """运动模型 - 在机器人局部坐标系下"""
        x_new = x.copy()
        x_new[0] += u[0] * math.cos(x[2]) * dt
        x_new[1] += u[0] * math.sin(x[2]) * dt
        x_new[2] += u[1] * dt
        return x_new
    
    def generate_trajectory_local(self, v, w, predict_time, dt):
        """
        生成局部坐标系下的轨迹
        起点固定为 (0, 0, 0)，即机器人当前位置
        """
        x = [0.0, 0.0, 0.0]  # 局部坐标系下机器人在原点
        trajectory = [x.copy()]
        time = 0
        while time <= predict_time:
            x = self.motion_model(x, [v, w], dt)
            trajectory.append(x.copy())
            time += dt
        return np.array(trajectory)
    
    def local_to_global(self, local_points, robot_pose):
        """
        将局部坐标系的点转换到全局坐标系
        robot_pose: [x, y, theta] 机器人在全局坐标系的位姿
        """
        rx, ry, rtheta = robot_pose[0], robot_pose[1], robot_pose[2]
        cos_theta = math.cos(rtheta)
        sin_theta = math.sin(rtheta)
        
        global_points = []
        for p in local_points:
            # 旋转 + 平移
            gx = p[0] * cos_theta - p[1] * sin_theta + rx
            gy = p[0] * sin_theta + p[1] * cos_theta + ry
            gtheta = p[2] + rtheta if len(p) > 2 else 0
            global_points.append([gx, gy, gtheta])
        
        return np.array(global_points)
    
    def get_path_direction_at_robot(self, current_pose, global_plan):
        """
        获取机器人位置处的路径切线方向
        """
        if global_plan is None or len(global_plan.poses) < 2:
            return current_pose[2]
        
        robot_x, robot_y = current_pose[0], current_pose[1]
        
        # 找最近点
        min_dist = float('inf')
        closest_idx = 0
        
        for i, pose in enumerate(global_plan.poses):
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            dist = dx*dx + dy*dy  # 不开方，节省计算
            if dist < min_dist:
                min_dist = dist
                closest_idx = i
        
        # 从最近点向前找一个合适的目标点（约2米远）
        target_idx = closest_idx
        for i in range(closest_idx, len(global_plan.poses)):
            pose = global_plan.poses[i]
            dx = pose.pose.position.x - robot_x
            dy = pose.pose.position.y - robot_y
            dist = math.sqrt(dx*dx + dy*dy)
            if dist >= 2.0:  # 2米前方
                target_idx = i
                break
            target_idx = i
        
        # 计算方向
        if target_idx > closest_idx:
            p1 = global_plan.poses[closest_idx]
            p2 = global_plan.poses[target_idx]
            dx = p2.pose.position.x - p1.pose.position.x
            dy = p2.pose.position.y - p1.pose.position.y
            
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                return math.atan2(dy, dx)
        
        # 后备：直接朝向前方点
        if target_idx < len(global_plan.poses):
            target = global_plan.poses[target_idx]
            dx = target.pose.position.x - robot_x
            dy = target.pose.position.y - robot_y
            if abs(dx) > 0.01 or abs(dy) > 0.01:
                return math.atan2(dy, dx)
        
        return current_pose[2]
    
    def calculate_path_direction_cost(self, trajectory_global, target_direction):
        """
        计算路径方向代价
        """
        if len(trajectory_global) == 0:
            return 1.0
        
        # 轨迹终点朝向与目标方向的差异
        final_theta = trajectory_global[-1][2]
        angle_diff = target_direction - final_theta
        angle_diff = math.atan2(math.sin(angle_diff), math.cos(angle_diff))
        
        return abs(angle_diff) / math.pi
    
    def calculate_smoothness_cost(self, v, w):
        """计算平滑性代价"""
        v_change = abs(v - self.last_v) / max(self.max_vel_x, 0.1)
        w_change = abs(w - self.last_w) / max(self.max_vel_theta, 0.1)
        angular_penalty = abs(w) / self.max_vel_theta
        
        return 0.2 * v_change + 0.4 * w_change + 0.4 * angular_penalty
    
    def calculate_velocity_cost(self, v):
        """计算速度代价"""
        return (self.max_vel_x - v) / self.max_vel_x
    
    def get_obstacles_local(self, costmap):
        """
        从 costmap 获取障碍物（局部坐标系）
        costmap 是以机器人为中心的，所以直接使用
        """
        obstacles = []
        if costmap is None:
            return obstacles
        
        width = costmap.info.width
        height = costmap.info.height
        resolution = costmap.info.resolution
        origin_x = costmap.info.origin.position.x
        origin_y = costmap.info.origin.position.y
        
        # costmap 中心就是机器人位置
        # 只搜索中心附近
        center_i = width // 2
        center_j = height // 2
        search_radius = int(self.obstacle_check_radius / resolution)
        
        i_min = max(0, center_i - search_radius)
        i_max = min(width, center_i + search_radius)
        j_min = max(0, center_j - search_radius)
        j_max = min(height, center_j + search_radius)
        
        step = 3  # 降采样
        
        for i in range(i_min, i_max, step):
            for j in range(j_min, j_max, step):
                index = j * width + i
                if index < len(costmap.data) and costmap.data[index] > 50:
                    # 转换到局部坐标（以机器人为中心）
                    ox = origin_x + (i + 0.5) * resolution
                    oy = origin_y + (j + 0.5) * resolution
                    obstacles.append([ox, oy])
        
        return obstacles
    
    def calculate_obstacle_cost_local(self, trajectory_local, obstacles_local):
        """
        计算障碍物代价（都在局部坐标系下）
        """
        if len(obstacles_local) == 0:
            return 0.0
        
        obstacles = np.array(obstacles_local)
        check_points = trajectory_local[::2, :2]
        
        min_distance = float('inf')
        for point in check_points:
            distances = np.sqrt(np.sum((obstacles - point)**2, axis=1))
            point_min_dist = np.min(distances)
            if point_min_dist < min_distance:
                min_distance = point_min_dist
        
        if min_distance <= self.obstacle_radius:
            return float('inf')
        
        return 1.0 / (min_distance + 0.1)
    
    def plan(self, current_pose, goal_pose, global_plan, costmap=None):
        """执行DWA规划"""
        current_pose = list(current_pose)
        
        # 获取局部坐标系下的障碍物
        obstacles_local = self.get_obstacles_local(costmap)
        
        # 获取路径方向（全局坐标系）
        target_direction = self.get_path_direction_at_robot(current_pose, global_plan)
        
        # 计算当前朝向与目标方向的差异
        current_theta = current_pose[2]
        direction_error = target_direction - current_theta
        direction_error = math.atan2(math.sin(direction_error), math.cos(direction_error))
        
        best_trajectory_local = None
        min_cost = float('inf')
        best_velocity = [0.0, 0.0]
        
        # 速度采样
        v_samples = np.arange(0.15, self.max_vel_x + self.vel_resolution, self.vel_resolution)
        
        # 角速度采样：根据方向误差偏向一侧
        # 如果需要左转（error > 0），多采样正角速度
        if direction_error > 0.1:
            w_samples = np.arange(0, self.max_vel_theta + 0.05, 0.05)
        elif direction_error < -0.1:
            w_samples = np.arange(-self.max_vel_theta, 0.05, 0.05)
        else:
            # 基本直行，小范围采样
            w_samples = np.arange(-0.15, 0.2, 0.05)
        
        for v in v_samples:
            for w in w_samples:
                # 生成局部轨迹
                trajectory_local = self.generate_trajectory_local(v, w, self.predict_time, self.dt)
                
                # 转换到全局坐标系计算方向代价
                trajectory_global = self.local_to_global(trajectory_local, current_pose)
                
                # 计算各项代价
                path_dir_cost = self.calculate_path_direction_cost(trajectory_global, target_direction)
                velocity_cost = self.calculate_velocity_cost(v)
                obstacle_cost = self.calculate_obstacle_cost_local(trajectory_local, obstacles_local)
                smoothness_cost = self.calculate_smoothness_cost(v, w)
                
                if obstacle_cost == float('inf'):
                    continue
                
                total_cost = (self.path_direction_weight * path_dir_cost +
                            self.velocity_weight * velocity_cost +
                            self.obstacle_weight * obstacle_cost +
                            self.smoothness_weight * smoothness_cost)
                
                if total_cost < min_cost:
                    min_cost = total_cost
                    best_trajectory_local = trajectory_local
                    best_velocity = [v, w]
        
        # 后备
        if best_trajectory_local is None:
            best_velocity = [0.2, 0.0]
            best_trajectory_local = self.generate_trajectory_local(0.2, 0.0, self.predict_time, self.dt)
        
        # 低通滤波
        filtered_v = self.velocity_filter_alpha * best_velocity[0] + \
                     (1 - self.velocity_filter_alpha) * self.last_v
        filtered_w = self.velocity_filter_alpha * best_velocity[1] + \
                     (1 - self.velocity_filter_alpha) * self.last_w
        
        self.last_v = filtered_v
        self.last_w = filtered_w
        
        # 用滤波后的速度重新生成轨迹，并转换到全局坐标系（用于 RViz 显示）
        trajectory_local = self.generate_trajectory_local(filtered_v, filtered_w, self.predict_time, self.dt)
        trajectory_global = self.local_to_global(trajectory_local, current_pose)
        
        return [filtered_v, filtered_w], trajectory_global
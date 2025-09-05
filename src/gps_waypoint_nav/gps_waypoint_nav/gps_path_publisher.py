#!/usr/bin/env python3
"""
GPS Path Publisher for ROS2 Navigation
发布预先记录的GPS路径用于Nav2全局路径规划
支持多种坐标系转换和路径优化
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header, ColorRGBA
import json
import csv
import numpy as np
from pathlib import Path as FilePath
import utm
from scipy.spatial.transform import Rotation as R
import tf2_ros
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

class GPSPathPublisher(Node):
    def __init__(self):
        super().__init__('gps_path_publisher')
        
        # 声明参数
        self.declare_parameter('gps_file', 'gps_path.json')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('publish_rate', 1.0)
        self.declare_parameter('use_utm', True)
        self.declare_parameter('loop_path', False)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('path_color', [0.0, 1.0, 0.0, 0.8])  # RGBA
        
        # 获取参数
        self.gps_file = self.get_parameter('gps_file').value
        self.frame_id = self.get_parameter('frame_id').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.use_utm = self.get_parameter('use_utm').value
        self.loop_path = self.get_parameter('loop_path').value
        self.publish_markers = self.get_parameter('publish_markers').value
        self.path_color = self.get_parameter('path_color').value
        
        # 加载GPS数据
        self.gps_points = []
        self.cartesian_points = []
        self.load_gps_data()
        
        # 如果使用UTM，转换GPS到笛卡尔坐标
        if self.use_utm and self.gps_points:
            self.convert_to_cartesian()
        
        # 发布者
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/reference_gps_path', 10)
        
        if self.publish_markers:
            self.marker_pub = self.create_publisher(MarkerArray, '/gps_path_markers', 10)
        
        # TF2
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 定时器
        self.timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_path_callback
        )
        
        self.get_logger().info(f'GPS路径发布节点启动')
        self.get_logger().info(f'  GPS文件: {self.gps_file}')
        self.get_logger().info(f'  路径点数: {len(self.gps_points)}')
        self.get_logger().info(f'  坐标系: {self.frame_id}')
        self.get_logger().info(f'  使用UTM: {self.use_utm}')

    def load_gps_data(self):
        """加载GPS数据文件"""
        file_path = FilePath(self.gps_file)
        
        if not file_path.exists():
            self.get_logger().error(f'GPS文件不存在: {self.gps_file}')
            return
        
        try:
            if file_path.suffix == '.json':
                self.load_json()
            elif file_path.suffix == '.csv':
                self.load_csv()
            elif file_path.suffix == '.yaml':
                self.load_yaml()
            else:
                self.get_logger().error(f'不支持的文件格式: {file_path.suffix}')
                
        except Exception as e:
            self.get_logger().error(f'加载GPS文件失败: {e}')

    def load_json(self):
        """加载JSON格式的GPS数据"""
        with open(self.gps_file, 'r') as f:
            data = json.load(f)
        
        if 'path' in data:
            self.gps_points = data['path']
        else:
            self.gps_points = data
        
        self.get_logger().info(f'从JSON加载了 {len(self.gps_points)} 个GPS点')

    def load_csv(self):
        """加载CSV格式的GPS数据"""
        with open(self.gps_file, 'r') as f:
            reader = csv.DictReader(f)
            for row in reader:
                point = {
                    'latitude': float(row['latitude']),
                    'longitude': float(row['longitude']),
                    'altitude': float(row.get('altitude', 0.0))
                }
                self.gps_points.append(point)
        
        self.get_logger().info(f'从CSV加载了 {len(self.gps_points)} 个GPS点')

    def load_yaml(self):
        """加载YAML格式的GPS数据"""
        # 简单的YAML解析（不使用外部库）
        points = []
        with open(self.gps_file, 'r') as f:
            lines = f.readlines()
            
        current_point = {}
        for line in lines:
            line = line.strip()
            if 'latitude:' in line:
                current_point['latitude'] = float(line.split(':')[1])
            elif 'longitude:' in line:
                current_point['longitude'] = float(line.split(':')[1])
            elif 'altitude:' in line:
                current_point['altitude'] = float(line.split(':')[1])
                # 假设altitude是最后一个字段
                if 'latitude' in current_point and 'longitude' in current_point:
                    points.append(current_point.copy())
                    current_point = {}
        
        self.gps_points = points
        self.get_logger().info(f'从YAML加载了 {len(self.gps_points)} 个GPS点')

    def convert_to_cartesian(self):
        """将GPS坐标转换为笛卡尔坐标（UTM）"""
        if not self.gps_points:
            return
        
        # 使用第一个点作为参考点
        first_point = self.gps_points[0]
        
        # 转换第一个点到UTM获取区域信息
        ref_x, ref_y, zone_number, zone_letter = utm.from_latlon(
            first_point['latitude'], 
            first_point['longitude']
        )
        
        self.get_logger().info(f'UTM参考点: Zone {zone_number}{zone_letter}, X={ref_x:.2f}, Y={ref_y:.2f}')
        
        # 转换所有点
        self.cartesian_points = []
        for gps_point in self.gps_points:
            try:
                # 转换到UTM坐标
                x, y, _, _ = utm.from_latlon(
                    gps_point['latitude'],
                    gps_point['longitude'],
                    force_zone_number=zone_number,
                    force_zone_letter=zone_letter
                )
                
                # 相对于第一个点的坐标
                rel_x = x - ref_x
                rel_y = y - ref_y
                rel_z = gps_point.get('altitude', 0.0) - first_point.get('altitude', 0.0)
                
                self.cartesian_points.append({
                    'x': rel_x,
                    'y': rel_y,
                    'z': rel_z,
                    'latitude': gps_point['latitude'],
                    'longitude': gps_point['longitude'],
                    'altitude': gps_point.get('altitude', 0.0)
                })
                
            except Exception as e:
                self.get_logger().warn(f'转换GPS点失败: {e}')
        
        self.get_logger().info(f'转换了 {len(self.cartesian_points)} 个点到笛卡尔坐标')

    def create_path_message(self):
        """创建Path消息"""
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        
        if self.use_utm and self.cartesian_points:
            # 使用笛卡尔坐标
            for i, point in enumerate(self.cartesian_points):
                pose = PoseStamped()
                pose.header.stamp = path_msg.header.stamp
                pose.header.frame_id = self.frame_id
                
                pose.pose.position.x = point['x']
                pose.pose.position.y = point['y']
                pose.pose.position.z = point['z']
                
                # 计算朝向（指向下一个点）
                if i < len(self.cartesian_points) - 1:
                    next_point = self.cartesian_points[i + 1]
                    dx = next_point['x'] - point['x']
                    dy = next_point['y'] - point['y']
                    yaw = np.arctan2(dy, dx)
                else:
                    # 最后一个点使用前一个点的朝向
                    if i > 0:
                        prev_point = self.cartesian_points[i - 1]
                        dx = point['x'] - prev_point['x']
                        dy = point['y'] - prev_point['y']
                        yaw = np.arctan2(dy, dx)
                    else:
                        yaw = 0.0
                
                # 转换为四元数
                q = R.from_euler('z', yaw).as_quat()
                pose.pose.orientation.x = q[0]
                pose.pose.orientation.y = q[1]
                pose.pose.orientation.z = q[2]
                pose.pose.orientation.w = q[3]
                
                path_msg.poses.append(pose)
        
        else:
            # 直接使用GPS坐标（需要其他节点进行转换）
            for i, gps_point in enumerate(self.gps_points):
                pose = PoseStamped()
                pose.header.stamp = path_msg.header.stamp
                pose.header.frame_id = self.frame_id
                
                # 注意：这里直接使用经纬度作为x,y，实际使用时需要适当的转换
                pose.pose.position.x = gps_point['longitude']
                pose.pose.position.y = gps_point['latitude']
                pose.pose.position.z = gps_point.get('altitude', 0.0)
                
                # 简单的朝向设置
                pose.pose.orientation.w = 1.0
                
                path_msg.poses.append(pose)
        
        # 如果需要闭环路径
        if self.loop_path and len(path_msg.poses) > 2:
            # 添加返回起点的路径
            first_pose = path_msg.poses[0]
            path_msg.poses.append(first_pose)
        
        return path_msg

    def create_marker_array(self):
        """创建可视化标记"""
        marker_array = MarkerArray()
        
        # 路径线标记
        path_marker = Marker()
        path_marker.header.stamp = self.get_clock().now().to_msg()
        path_marker.header.frame_id = self.frame_id
        path_marker.ns = "gps_path"
        path_marker.id = 0
        path_marker.type = Marker.LINE_STRIP
        path_marker.action = Marker.ADD
        
        path_marker.scale.x = 0.2  # 线宽
        path_marker.color.r = self.path_color[0]
        path_marker.color.g = self.path_color[1]
        path_marker.color.b = self.path_color[2]
        path_marker.color.a = self.path_color[3]
        
        if self.use_utm and self.cartesian_points:
            for point in self.cartesian_points:
                p = Point()
                p.x = point['x']
                p.y = point['y']
                p.z = point['z']
                path_marker.points.append(p)
        else:
            for gps_point in self.gps_points:
                p = Point()
                p.x = gps_point['longitude']
                p.y = gps_point['latitude']
                p.z = gps_point.get('altitude', 0.0)
                path_marker.points.append(p)
        
        marker_array.markers.append(path_marker)
        
        # 点标记
        points_marker = Marker()
        points_marker.header = path_marker.header
        points_marker.ns = "gps_points"
        points_marker.id = 1
        points_marker.type = Marker.SPHERE_LIST
        points_marker.action = Marker.ADD
        
        points_marker.scale.x = 0.5
        points_marker.scale.y = 0.5
        points_marker.scale.z = 0.5
        
        points_marker.color.r = 1.0
        points_marker.color.g = 0.0
        points_marker.color.b = 0.0
        points_marker.color.a = 0.8
        
        # 使用相同的点
        points_marker.points = path_marker.points
        
        # 为每个点设置不同的颜色（可选）
        for i in range(len(points_marker.points)):
            color = ColorRGBA()
            # 从绿色渐变到红色
            ratio = i / max(len(points_marker.points) - 1, 1)
            color.r = ratio
            color.g = 1.0 - ratio
            color.b = 0.0
            color.a = 0.8
            points_marker.colors.append(color)
        
        marker_array.markers.append(points_marker)
        
        # 起点和终点特殊标记
        if len(path_marker.points) > 0:
            # 起点标记
            start_marker = Marker()
            start_marker.header = path_marker.header
            start_marker.ns = "start_point"
            start_marker.id = 2
            start_marker.type = Marker.SPHERE
            start_marker.action = Marker.ADD
            start_marker.pose.position = path_marker.points[0]
            start_marker.scale.x = 1.0
            start_marker.scale.y = 1.0
            start_marker.scale.z = 1.0
            start_marker.color.r = 0.0
            start_marker.color.g = 1.0
            start_marker.color.b = 0.0
            start_marker.color.a = 1.0
            marker_array.markers.append(start_marker)
            
            # 终点标记
            end_marker = Marker()
            end_marker.header = path_marker.header
            end_marker.ns = "end_point"
            end_marker.id = 3
            end_marker.type = Marker.SPHERE
            end_marker.action = Marker.ADD
            end_marker.pose.position = path_marker.points[-1]
            end_marker.scale.x = 1.0
            end_marker.scale.y = 1.0
            end_marker.scale.z = 1.0
            end_marker.color.r = 1.0
            end_marker.color.g = 0.0
            end_marker.color.b = 0.0
            end_marker.color.a = 1.0
            marker_array.markers.append(end_marker)
        
        return marker_array

    def publish_gps_points(self):
        """发布原始GPS点（NavSatFix格式）"""
        for gps_point in self.gps_points:
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            
            msg.latitude = gps_point['latitude']
            msg.longitude = gps_point['longitude']
            msg.altitude = gps_point.get('altitude', 0.0)
            
            # 设置状态（假设都是有效的）
            msg.status.status = 0  # STATUS_FIX
            msg.status.service = 1  # SERVICE_GPS
            
            self.gps_pub.publish(msg)
            
            # 发布间隔（避免一次发送太多消息）
            rclpy.spin_once(self, timeout_sec=0.001)

    def publish_path_callback(self):
        """定时发布路径"""
        # 发布Path消息
        path_msg = self.create_path_message()
        self.path_pub.publish(path_msg)
        
        # 发布可视化标记
        if self.publish_markers:
            marker_array = self.create_marker_array()
            self.marker_pub.publish(marker_array)
        
        # 定期发布一些GPS点（可选）
        if len(self.gps_points) > 0:
            # 每次发布一个GPS点（循环）
            if not hasattr(self, 'current_gps_index'):
                self.current_gps_index = 0
            
            gps_point = self.gps_points[self.current_gps_index]
            msg = NavSatFix()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "gps"
            msg.latitude = gps_point['latitude']
            msg.longitude = gps_point['longitude']
            msg.altitude = gps_point.get('altitude', 0.0)
            msg.status.status = 0
            msg.status.service = 1
            
            self.gps_pub.publish(msg)
            
            self.current_gps_index = (self.current_gps_index + 1) % len(self.gps_points)
        
        # 日志（降低频率）
        if not hasattr(self, 'log_counter'):
            self.log_counter = 0
        self.log_counter += 1
        
        if self.log_counter % int(self.publish_rate * 10) == 0:  # 每10秒一次
            self.get_logger().info(
                f'发布GPS路径: {len(path_msg.poses)} 个位姿点'
            )


def main(args=None):
    rclpy.init(args=args)
    
    node = GPSPathPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('节点被用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


# ros2 launch gps_waypoint_nav gps_waypoint_nav.launch.py \
#   gps_file:=~/ros2_ws/src/gps_waypoint_nav/data/orchard_path.json
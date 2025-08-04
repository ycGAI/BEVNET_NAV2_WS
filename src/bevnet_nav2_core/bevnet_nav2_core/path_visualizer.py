#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import numpy as np

class PathVisualizer(Node):
    def __init__(self):
        super().__init__('path_visualizer')
        
        # 订阅路径
        self.global_path_sub = self.create_subscription(
            Path, '/plan', self.global_path_callback, 10)
        self.local_path_sub = self.create_subscription(
            Path, '/local_plan', self.local_path_callback, 10)
        
        # 发布可视化
        self.marker_pub = self.create_publisher(
            MarkerArray, '/path_visualization', 10)
        
        # 存储路径
        self.global_path = None
        self.local_path = None
        
        # 定时更新可视化
        self.timer = self.create_timer(0.1, self.update_visualization)
    
    def create_path_marker(self, path, color, marker_id, width=0.1):
        """创建路径标记"""
        marker = Marker()
        marker.header = path.header
        marker.ns = 'path'
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.scale.x = width
        marker.color = color
        marker.pose.orientation.w = 1.0
        
        # 添加路径点
        for pose in path.poses:
            point = Point()
            point.x = pose.pose.position.x
            point.y = pose.pose.position.y
            point.z = pose.pose.position.z + 0.1
            marker.points.append(point)
        
        return marker
    
    def update_visualization(self):
        """更新路径可视化"""
        markers = MarkerArray()
        
        # 全局路径（蓝色）
        if self.global_path:
            global_color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.7)
            markers.markers.append(
                self.create_path_marker(
                    self.global_path, global_color, 0, 0.05))
        
        # 局部路径（绿色，更粗）
        if self.local_path:
            local_color = ColorRGBA(r=0.0, g=1.0, b=0.0, a=1.0)
            markers.markers.append(
                self.create_path_marker(
                    self.local_path, local_color, 1, 0.1))
        
        # 发布
        if markers.markers:
            self.marker_pub.publish(markers)
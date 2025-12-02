#!/usr/bin/env python3
"""
BEVNet Inference ROS2 Node - 支持行人检测
"""
import sys
import os
sys.path.insert(0, '/workspace/bevnet')

import rclpy
from rclpy.node import Node
import numpy as np
import torch
import cv2
from sensor_msgs.msg import PointCloud2, Image, PointField
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
from cv_bridge import CvBridge
import struct
import argparse

from bevnet.inference import BEVNetSingle, BEVNetSingleWithSafety, create_bevnet_model
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy


class BEVNetInferenceNode(Node):
    def __init__(self, model_path, model_type='single', with_safety=False,
                 safety_radius=1.5, human_confidence=10.0):
        super().__init__('bevnet_inference_node')
        
        # 参数
        self.model_path = model_path
        self.model_type = model_type
        self.with_safety = with_safety
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # 加载模型
        self.get_logger().info(f'Loading model from: {model_path}')
        self.get_logger().info(f'Model type: {model_type}')
        self.get_logger().info(f'Human detection: {"ENABLED" if with_safety else "DISABLED"}')
        self.get_logger().info(f'Device: {self.device}')
        
        try:
            if with_safety:
                # 人员检测配置
                human_detection_config = {
                    'enabled': True,
                    'height_range': (1.0, 2.5),
                    'width_range': (0.2, 1.2),
                    'min_points': 20,
                    'ground_threshold': 0.2,
                    'clustering_eps': 0.6,
                    'clustering_min_samples': 5
                }
                
                # 安全配置
                safety_config = {
                    'safety_radius': safety_radius,
                    'human_confidence': human_confidence,
                    'human_class': None
                }
                
                self.model = create_bevnet_model(
                    model_path,
                    device=self.device,
                    with_safety=True,
                    human_detection_config=human_detection_config,
                    safety_config=safety_config,
                    model_type=model_type
                )
                self.get_logger().info(f'Safety config: radius={safety_radius}m, confidence={human_confidence}')
            else:
                self.model = create_bevnet_model(
                    model_path,
                    device=self.device,
                    with_safety=False,
                    model_type=model_type
                )
            
            self.get_logger().info('Model loaded successfully!')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
            return
        
        # ROS2设置
        self.bridge = CvBridge()
        
        # QoS配置
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 订阅点云
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            qos_profile
        )
        
        # 发布器
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/bevnet/costmap', 10)
        self.semantic_pub = self.create_publisher(Image, '/bevnet/semantic_image', 10)
        
        # BEV参数
        self.bev_resolution = 0.05
        self.bev_width = 407
        self.bev_height = 407
        self.bev_x_range = (-10.2, 10.2)
        self.bev_y_range = (-10.2, 10.2)
        
        # 统计
        self.frame_count = 0
        self.human_detection_count = 0
        
        self.get_logger().info('BEVNet inference node ready!')
    
    def pointcloud_callback(self, msg):
        """处理点云消息"""
        try:
            # 解析点云
            points = self.parse_pointcloud(msg)
            
            if len(points) == 0:
                self.get_logger().warn('Empty point cloud received')
                return
            
            self.frame_count += 1
            
            # BEVNet推理
            if self.with_safety:
                # 带行人检测的推理
                output, human_positions = self.model.predict(points, return_human_positions=True)
                
                if human_positions:
                    self.human_detection_count += len(human_positions)
                    self.get_logger().info(f'Frame {self.frame_count}: Detected {len(human_positions)} humans at {human_positions}')
            else:
                # 普通推理
                output = self.model.predict(points)
            
            # 将CUDA张量转换为CPU numpy
            if torch.is_tensor(output):
                output = output.cpu().numpy()
            
            # 处理输出维度
            if len(output.shape) == 4 and output.shape[0] == 1:
                output = output.squeeze(0)
            
            # 发布结果
            self.publish_costmap(output, msg.header)
            self.publish_semantic_image(output, msg.header)
            
            if self.frame_count % 50 == 0:
                self.get_logger().info(f'Processed {self.frame_count} frames, total humans detected: {self.human_detection_count}')
        
        except Exception as e:
            self.get_logger().error(f'Processing failed: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
        
    def parse_pointcloud(self, msg):
        """解析PointCloud2消息为numpy数组"""
        points = []
        point_step = msg.point_step
        
        for i in range(0, len(msg.data), point_step):
            if i + 16 <= len(msg.data):
                x = struct.unpack('f', msg.data[i:i+4])[0]
                y = struct.unpack('f', msg.data[i+4:i+8])[0] 
                z = struct.unpack('f', msg.data[i+8:i+12])[0]
                intensity = struct.unpack('f', msg.data[i+12:i+16])[0]
                
                if not np.isnan(x) and not np.isnan(y) and not np.isnan(z):
                    points.append([x, y, z, intensity])
        
        return np.array(points, dtype=np.float32)
    
    def publish_costmap(self, bev_output, header):
        """发布OccupancyGrid格式的代价地图"""
        costmap = OccupancyGrid()
        costmap.header = header
        costmap.header.frame_id = "velodyne"
        
        costmap.info.resolution = self.bev_resolution
        costmap.info.width = self.bev_width
        costmap.info.height = self.bev_height
        
        costmap.info.origin.position.x = self.bev_x_range[0]
        costmap.info.origin.position.y = self.bev_y_range[0]
        costmap.info.origin.position.z = -1.5
        costmap.info.origin.orientation.w = 1.0
        
        if len(bev_output.shape) == 3:
            bev_map = np.argmax(bev_output, axis=0)
        else:
            bev_map = bev_output
        
        # 映射到代价值
        cost_map = np.zeros_like(bev_map, dtype=np.int8)
        cost_map[bev_map == 0] = 0     # free
        cost_map[bev_map == 1] = 10    # road
        cost_map[bev_map == 2] = 50    # medium cost
        cost_map[bev_map == 3] = 100   # obstacle / human (高危险)
        cost_map[bev_map == 4] = -1    # unknown
        
        costmap.data = cost_map.flatten().tolist()
        self.costmap_pub.publish(costmap)
    
    def publish_semantic_image(self, bev_output, header):
        """发布语义分割的彩色图像"""
        # 颜色映射 - 添加人员检测的颜色
        colors = [
            [0, 255, 0],      # 绿色 - free space
            [128, 128, 128],  # 灰色 - road
            [255, 165, 0],    # 橙色 - medium cost (vegetation)
            [255, 0, 0],      # 红色 - obstacle / human
            [0, 0, 255],      # 蓝色 - unknown
        ]
        
        if len(bev_output.shape) == 3:
            bev_map = np.argmax(bev_output, axis=0)
        else:
            bev_map = bev_output
        
        color_image = np.zeros((self.bev_height, self.bev_width, 3), dtype=np.uint8)
        for i, color in enumerate(colors):
            if i < np.max(bev_map) + 1:
                color_image[bev_map == i] = color
        
        img_msg = self.bridge.cv2_to_imgmsg(color_image, "rgb8")
        img_msg.header = header
        self.semantic_pub.publish(img_msg)


def main(args=None):
    parser = argparse.ArgumentParser(description='BEVNet Inference Node with Optional Human Detection')
    parser.add_argument('model_path', type=str, help='Path to model file')
    parser.add_argument('--model_type', '-t', type=str, default='single', 
                        choices=['single', 'recurrent'],
                        help='Model type (default: single)')
    parser.add_argument('--with-safety', '-s', action='store_true',
                        help='Enable human detection and safety features')
    parser.add_argument('--safety-radius', '-r', type=float, default=1.5,
                        help='Safety radius around detected humans in meters (default: 1.5)')
    parser.add_argument('--human-confidence', '-c', type=float, default=10.0,
                        help='Confidence value for human detection (default: 10.0)')
    
    # 解析参数（排除ROS2参数）
    parsed_args, remaining = parser.parse_known_args()
    
    # 初始化ROS2
    rclpy.init(args=remaining)
    
    # 创建节点
    node = BEVNetInferenceNode(
        model_path=parsed_args.model_path,
        model_type=parsed_args.model_type,
        with_safety=parsed_args.with_safety,
        safety_radius=parsed_args.safety_radius,
        human_confidence=parsed_args.human_confidence
    )
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
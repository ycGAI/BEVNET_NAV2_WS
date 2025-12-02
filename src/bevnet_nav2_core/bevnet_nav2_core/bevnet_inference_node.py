#!/usr/bin/env python3
"""
BEVNet Inference ROS2 Node
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

from bevnet.inference import BEVNetSingle, BEVNetRecurrent
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

class BEVNetInferenceNode(Node):
    def __init__(self, model_path, model_type='single'):
        super().__init__('bevnet_inference_node')
        
        # 参数
        self.model_path = model_path
        self.model_type = model_type
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        
        # 加载模型
        self.get_logger().info(f'Loading model from: {model_path}')
        self.get_logger().info(f'Model type: {model_type}')
        self.get_logger().info(f'Device: {self.device}')
        
        try:
            if model_type == 'single':
                self.model = BEVNetSingle(model_path, device=self.device)
            else:
                self.model = BEVNetRecurrent(model_path, device=self.device)
            self.get_logger().info('Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            return
        
        # ROS2设置
        self.bridge = CvBridge()
        
        # 订阅点云
        # self.pc_sub = self.create_subscription(
        #     PointCloud2,
        #     '/velodyne_points',
        #     self.pointcloud_callback,
        #     10
        # )
        # 配置QoS - 匹配发布器的BEST_EFFORT
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # 订阅点云 - 使用兼容的QoS
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            qos_profile  # 使用BEST_EFFORT而不是默认的RELIABLE
        )
        
        # 发布器
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/bevnet/costmap', 10)
        self.semantic_pub = self.create_publisher(Image, '/bevnet/semantic_image', 10)
        
        # BEV参数
        self.bev_resolution = 0.05 # 米/像素
        self.bev_width = 407
        self.bev_height = 407
        self.bev_x_range = (-10.2, 10.2)  # 米
        self.bev_y_range = (-10.2, 10.2)  # 米
        
        self.get_logger().info('BEVNet inference node ready!')
    
    def pointcloud_callback(self, msg):
        """处理点云消息"""
        try:
            # 解析点云
            points = self.parse_pointcloud(msg)
            
            if len(points) == 0:
                self.get_logger().warn('Empty point cloud received')
                return
            
            self.get_logger().info(f'Processing {len(points)} points...')
            
            # BEVNet推理
            output = self.model.predict(points)
            
            # 调试：打印输出形状
            self.get_logger().info(f'Model output shape: {output.shape}')
            
            # 将CUDA张量转换为CPU numpy
            if torch.is_tensor(output):
                output = output.cpu().numpy()
            
            # 处理输出维度
            # BEVNet输出形状: [n, num_classes, H, W]，其中n=1
            if len(output.shape) == 4 and output.shape[0] == 1:
                # 去掉批次维度: [1, C, H, W] -> [C, H, W]
                output = output.squeeze(0)
                self.get_logger().info(f'Squeezed output shape: {output.shape}')
            
            # 发布结果
            self.publish_costmap(output, msg.header)
            self.publish_semantic_image(output, msg.header)
            
            self.get_logger().info(f'Published costmap and semantic image')
        
        except Exception as e:
            self.get_logger().error(f'Processing failed: {e}')
            import traceback
            self.get_logger().error(f'Traceback:\n{traceback.format_exc()}')
        
    def parse_pointcloud(self, msg):
        """解析PointCloud2消息为numpy数组"""
        points = []
        
        # 获取点云格式信息
        point_step = msg.point_step
        
        # 解析二进制数据
        for i in range(0, len(msg.data), point_step):
            # KITTI格式：x, y, z, intensity
            if i + 16 <= len(msg.data):
                x = struct.unpack('f', msg.data[i:i+4])[0]
                y = struct.unpack('f', msg.data[i+4:i+8])[0] 
                z = struct.unpack('f', msg.data[i+8:i+12])[0]
                intensity = struct.unpack('f', msg.data[i+12:i+16])[0]
                
                # 过滤无效点
                if not np.isnan(x) and not np.isnan(y) and not np.isnan(z):
                    points.append([x, y, z, intensity])
        
        return np.array(points, dtype=np.float32)
    
    def publish_costmap(self, bev_output, header):
        """发布OccupancyGrid格式的代价地图"""
        # bev_output shape: [C, H, W]，其中C是类别数
        
        costmap = OccupancyGrid()
        costmap.header = header
        costmap.header.frame_id = "velodyne"  # 使用map而不是base_link
        
        costmap.info.resolution = self.bev_resolution
        costmap.info.width = self.bev_width
        costmap.info.height = self.bev_height
        
        # 设置原点（左下角）
        costmap.info.origin.position.x = self.bev_x_range[0]
        costmap.info.origin.position.y = self.bev_y_range[0]
        costmap.info.origin.position.z = -1.5
        costmap.info.origin.orientation.w = 1.0
        
        # 转换BEV输出为OccupancyGrid数据
        # bev_output shape: [C, H, W]
        if len(bev_output.shape) == 3:
            # 取最大概率的类别
            bev_map = np.argmax(bev_output, axis=0)  # [H, W]
        else:
            bev_map = bev_output
        
        # 映射到代价值
        # 根据您的模型，可能需要调整这些映射
        # 0: free space -> 0
        # 1: road -> 10  
        # 2: vegetation/medium -> 50
        # 3: obstacle -> 100
        # 4: unknown -> -1
        cost_map = np.zeros_like(bev_map, dtype=np.int8)
        cost_map[bev_map == 0] = 0    # free
        cost_map[bev_map == 1] = 10   # road (low cost)
        cost_map[bev_map == 2] = 50   # medium cost
        cost_map[bev_map == 3] = 100  # obstacle
        cost_map[bev_map == 4] = -1   # unknown
        
        # 展平并设置数据
        costmap.data = cost_map.flatten().tolist()
        
        self.costmap_pub.publish(costmap)
    
    def publish_semantic_image(self, bev_output, header):
        """发布语义分割的彩色图像"""
        # 颜色映射
        colors = [
            [0, 255, 0],      # 绿色 - free space
            [128, 128, 128],  # 灰色 - road
            [255, 0, 0],      # 红色 - obstacle
            [0, 0, 255],      # 蓝色 - unknown
        ]
        
        # 创建彩色图像
        if len(bev_output.shape) == 3:
            bev_map = np.argmax(bev_output, axis=0)
        else:
            bev_map = bev_output
        
        color_image = np.zeros((self.bev_height, self.bev_width, 3), dtype=np.uint8)
        for i, color in enumerate(colors):
            if i < np.max(bev_map) + 1:
                color_image[bev_map == i] = color
        
        # 转换为ROS消息
        img_msg = self.bridge.cv2_to_imgmsg(color_image, "rgb8")
        img_msg.header = header
        
        self.semantic_pub.publish(img_msg)

def main(args=None):
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('model_path', type=str, help='Path to model file')
    parser.add_argument('--model_type', type=str, default='single', choices=['single', 'recurrent'])
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    
    node = BEVNetInferenceNode(parsed_args.model_path, parsed_args.model_type)
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH
# ros2 run bevnet_nav2_core bevnet_inference_node.py /workspace/bevnet_nav2_ws/models/best.pth.34
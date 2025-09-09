#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/workspace/bevnet')

import numpy as np
import torch
import yaml
from bevnet.inference import BEVNetSingle, BEVNetRecurrent

# ROS2导入（可选）
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import PointCloud2, PointField, Image
    from nav_msgs.msg import OccupancyGrid
    from std_msgs.msg import Header
    from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
    import struct
    HAS_ROS = True
except ImportError:
    print("Warning: ROS2 not available, running in standalone mode")
    HAS_ROS = False

try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    print("Warning: cv_bridge not available")
    HAS_CV_BRIDGE = False

class BEVNetProcessor:
    """BEVNet处理器，可以独立于ROS使用"""
    def __init__(self, model_path, model_type='single', device='cuda'):
        # 加载模型
        print(f"Loading BEVNet model from: {model_path}")
        self.device = device if torch.cuda.is_available() else 'cpu'
        
        if model_type == 'single':
            self.model = BEVNetSingle(model_path, device=self.device)
        else:
            self.model = BEVNetRecurrent(model_path, device=self.device)
            
        print("Model loaded successfully!")
        
        # 从模型配置获取参数
        self.map_size = 407  # 从输出看到的尺寸
        self.map_resolution = 0.2
        self.num_classes = 5
        self.point_cloud_range = [-51.2, -51.2, -2, 51.2, 51.2, 1.0]
        
        # 语义到代价映射（ATB4数据集）
        self.semantic_to_cost = {
            0: 0,     # free space
            1: 30,    # terrain (low cost)
            2: 60,    # vegetation (medium cost)
            3: 100,   # obstacles (lethal)
            4: -1,    # unknown
        }
        
        # 颜色映射
        self.color_map = np.array([
            [0, 255, 0],      # 0: free (green)
            [255, 255, 0],    # 1: terrain (yellow)
            [0, 0, 255],      # 2: vegetation (blue)
            [255, 0, 0],      # 3: obstacles (red)
            [128, 128, 128],  # 4: unknown (gray)
        ], dtype=np.uint8)
        
    def process_pointcloud(self, points):
        """处理点云并返回语义地图"""
        # 运行推理
        output = self.model.predict(points)
        
        # 处理输出 - output shape: [1, 5, 407, 407]
        if torch.is_tensor(output):
            output = output.cpu().numpy()
        
        # 移除
        if len(output.shape) == 4:  # [B, C, H, W]
            output = output[0]  # [C, H, W]
        
        semantic_map = np.argmax(output, axis=0)  # [H, W]
        confidence = np.max(output, axis=0)  # [H, W]
        
        return semantic_map, confidence
    
    def create_costmap(self, semantic_map):
        """将语义地图转换为Nav2代价地图"""
        costmap = np.zeros_like(semantic_map, dtype=np.int8)
        
        for sem_class, cost in self.semantic_to_cost.items():
            if sem_class < self.num_classes:
                mask = semantic_map == sem_class
                costmap[mask] = cost
                
        return costmap
    
    def create_visualization(self, semantic_map):
        """创建彩色可视化"""
        h, w = semantic_map.shape
        color_image = np.zeros((h, w, 3), dtype=np.uint8)
        
        for class_id in range(self.num_classes):
            mask = semantic_map == class_id
            color_image[mask] = self.color_map[class_id]
            
        return color_image

if HAS_ROS:
    class BEVNetROS2Node(Node):
        """ROS2节点封装"""
        def __init__(self):
            super().__init__('bevnet_inference_node')
            
            # 声明参
            self.declare_parameter('model_path', '/workspace/bevnet_nav2_ws/models/best.pth.34')
            self.declare_parameter('model_type', 'single')
            self.declare_parameter('device', 'cuda')
            self.declare_parameter('pointcloud_topic', '/velodyne_points')
            self.declare_parameter('visualize', True)
            
            # 获取参数
            model_path = self.get_parameter('model_path').value
            model_type = self.get_parameter('model_type').value
            device = self.get_parameter('device').value
            pointcloud_topic = self.get_parameter('pointcloud_topic').value
            
            # 初始化处理器
            self.processor = BEVNetProcessor(model_path, model_type, device)
            
            # QoS配置
            qos = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                history=HistoryPolicy.KEEP_LAST,
                depth=1
            )
            
            # 订阅点云
            self.pc_sub = self.create_subscription(
                PointCloud2,
                pointcloud_topic,
                self.pointcloud_callback,
                qos
            )
            
            # 发布器
            self.costmap_pub = self.create_publisher(
                OccupancyGrid,
                '/bevnet/costmap',
                10
            )
            
            self.semantic_pub = self.create_publisher(
                OccupancyGrid,
                '/bevnet/semantic_map',
                10
            )
            
            if HAS_CV_BRIDGE and self.get_parameter('visualize').value:
                self.bridge = CvBridge()
                self.viz_pub = self.create_publisher(
                    Image,
                    '/bevnet/visualization',
                    10
                )
            else:
                self.viz_pub = None
            
            self.get_logger().info('BEVNet ROS2 node initialized')
            self.get_logger().info(f'Listening on: {pointcloud_topic}')
        
        def pointcloud_to_numpy(self, msg):
            """将PointCloud2转换为numpy数组"""
            # 解析点云
            points = []
            
            # 简单的解析方
            point_step = msg.point_step
            data = msg.data
            
            for i in range(0, len(data), point_step):
                if i + 16 <= len(data):
                    x = struct.unpack('f', data[i:i+4])[0]
                    y = struct.unpack('f', data[i+4:i+8])[0]
                    z = struct.unpack('f', data[i+8:i+12])[0]
                    intensity = struct.unpack('f', data[i+12:i+16])[0] if i+16 <= len(data) else 0.0
                    points.append([x, y, z, intensity])
            
            return np.array(points, dtype=np.float32)
        
        def pointcloud_callback(self, msg):
            """处理点云消息"""
            try:
                # 转换点云
                points = self.pointcloud_to_numpy(msg)
                if len(points) < 100:
                    self.get_logger().warn(f"Too few points: {len(points)}")
                    return
                
                # 处理点云
                semantic_map, confidence = self.processor.process_pointcloud(points)
                
                # 发布语义地图
                self.publish_semantic_map(semantic_map, msg.header)
                
                # 发布代价地图
                costmap = self.processor.create_costmap(semantic_map)
                self.publish_costmap(costmap, msg.header)
                
                # 发布可视化
                if self.viz_pub is not None:
                    color_image = self.processor.create_visualization(semantic_map)
                    self.publish_visualization(color_image, msg.header)
                
            except Exception as e:
                self.get_logger().error(f'Error in pointcloud callback: {e}')
                import traceback
                traceback.print_exc()
        
        def publish_semantic_map(self, semantic_map, header):
            """发布语义地图"""
            msg = OccupancyGrid()
            msg.header = header
            msg.header.frame_id = 'base_link'
            msg.info.resolution = self.processor.map_resolution
            msg.info.width = semantic_map.shape[1]
            msg.info.height = semantic_map.shape[0]
            msg.info.origin.position.x = self.processor.point_cloud_range[0]
            msg.info.origin.position.y = self.processor.point_cloud_range[1]
            msg.info.origin.orientation.w = 1.0
            
            # 0-100的值
            semantic_vis = (semantic_map.astype(np.float32) / (self.processor.num_classes - 1) * 100).astype(np.int8)
            msg.data = np.flipud(semantic_vis).flatten().tolist()
            
            self.semantic_pub.publish(msg)
        
        def publish_costmap(self, costmap, header):
            """发布Nav2代价地图"""
            msg = OccupancyGrid()
            msg.header = header
            msg.header.frame_id = 'base_link'
            msg.info.resolution = self.processor.map_resolution
            msg.info.width = costmap.shape[1]
            msg.info.height = costmap.shape[0]
            msg.info.origin.position.x = self.processor.point_cloud_range[0]
            msg.info.origin.position.y = self.processor.point_cloud_range[1]
            msg.info.origin.orientation.w = 1.0
            
            msg.data = np.flipud(costmap).flatten().tolist()
            self.costmap_pub.publish(msg)
        
        def publish_visualization(self, color_image, header):
            """发布可视化图像"""
            if self.viz_pub is not None and HAS_CV_BRIDGE:
                color_image_flipped = np.flipud(color_image)
                img_msg = self.bridge.cv2_to_imgmsg(color_image_flipped, encoding='bgr8')
                img_msg.header = header
                self.viz_pub.publish(img_msg)

def test_standalone():
    """独立测试"""
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument('--model_path', type=str, 
                       default='/workspace/bevnet_nav2_ws/models/best.pth.34')
    parser.add_argument('--save_output', action='store_true')
    args = parser.parse_args()
    
    # 创建处理器
    processor = BEVNetProcessor(args.model_path)
    
    # 创建测试点云
    print("\nCreating test point cloud...")
    num_points = 50000
    points = np.zeros((num_points, 4), dtype=np.float32)
    
    # 生成环形点云
    angles = np.random.uniform(0, 2*np.pi, num_points)
    distances = np.random.uniform(0.5, 30, num_points)
    heights = np.random.uniform(-2, 0.5, num_points)
    
    points[:, 0] = distances * np.cos(angles)
    points[:, 1] = distances * np.sin(angles)
    points[:, 2] = heights
    points[:, 3] = np.random.uniform(0, 1, num_points)
    
    # 处理
    print("Processing point cloud...")
    semantic_map, confidence = processor.process_pointcloud(points)
    costmap = processor.create_costmap(semantic_map)
    color_image = processor.create_visualization(semantic_map)
    
    print(f"Semantic map shape: {semantic_map.shape}")
    print(f"Unique classes: {np.unique(semantic_map)}")
    print(f"Class distribution: {[(i, np.sum(semantic_map==i)) for i in range(5)]}")
    
    # 保存结果
    if args.save_output:
        cv2.imwrite('/workspace/bevnet_nav2_ws/bevnet_semantic.png', color_image)
        print("Saved visualization to bevnet_semantic.png")

def main():
    if HAS_ROS and len(sys.argv) > 1 and '--ros' in sys.argv:
        # ROS2模式
        print("Starting BEVNet ROS2 node...")
        rclpy.init()
        node = BEVNetROS2Node()
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        # 独立模式
        print("Running in standalone mode (no ROS2)")
        test_standalone()

if __name__ == '__main__':
    main()

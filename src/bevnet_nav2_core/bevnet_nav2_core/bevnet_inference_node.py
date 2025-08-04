#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import numpy as np
import torch
import cv2
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import sys
import os
import yaml
import struct
from sensor_msgs_py import point_cloud2

# 添加bevnet到Python路径
sys.path.insert(0, '/workspace/bevnet')

# 导入BEVNet推理类
try:
    from bevnet.inference import BEVNetSingle, BEVNetRecurrent
except ImportError as e:
    print(f"Error importing bevnet inference: {e}")
    print("Make sure bevnet is properly installed and PYTHONPATH is set")
    raise

class BEVNetInferenceNode(Node):
    def __init__(self):
        super().__init__('bevnet_inference_node')
        
        # 声明参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('model_type', 'single')  # 'single' or 'recurrent'
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('visualize', True)
        self.declare_parameter('pointcloud_topic', '/velodyne_points')
        self.declare_parameter('use_safety_detection', False)
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        model_type = self.get_parameter('model_type').value
        device = self.get_parameter('device').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        use_safety = self.get_parameter('use_safety_detection').value
        
        # 验证模型路径
        if not model_path or not os.path.exists(model_path):
            self.get_logger().error(f'Model path not found: {model_path}')
            raise FileNotFoundError(f'Model file does not exist: {model_path}')
        
        # 加载模型
        self.get_logger().info(f'Loading BEVNet {model_type} model from: {model_path}')
        self.get_logger().info(f'Using device: {device}')
        
        try:
            # 根据模型类型选择推理类
            if model_type == 'single':
                self.model = BEVNetSingle(model_path, device=device)
            elif model_type == 'recurrent':
                self.model = BEVNetRecurrent(model_path, device=device)
            else:
                raise ValueError(f'Unknown model type: {model_type}')
            
            self.device = device
            self.model_type = model_type
            
            # 从模型配置中获取参数
            if hasattr(self.model, 'g') and self.model.g is not None:
                voxelizer_cfg = self.model.g.get('voxelizer', {})
                self.point_cloud_range = voxelizer_cfg.get('point_cloud_range', 
                                                          [-51.2, -51.2, -2, 51.2, 51.2, 1.0])
                self.voxel_size = voxelizer_cfg.get('voxel_size', [0.2, 0.2, 0.1])
                
                # 计算BEV地图参数
                self.map_size_x = int((self.point_cloud_range[3] - self.point_cloud_range[0]) / self.voxel_size[0])
                self.map_size_y = int((self.point_cloud_range[4] - self.point_cloud_range[1]) / self.voxel_size[1])
                self.map_resolution = self.voxel_size[0]  # 假设x和y分辨率相同
                
                # 获取类别数
                dataset_config = self.model.g.get('dataset_config', {})
                if isinstance(dataset_config, str) and os.path.exists(dataset_config):
                    with open(dataset_config, 'r') as f:
                        dataset_config = yaml.safe_load(f)
                self.num_classes = dataset_config.get('num_class', 5)
            else:
                # 使用默认值
                self.point_cloud_range = [-51.2, -51.2, -2, 51.2, 51.2, 1.0]
                self.voxel_size = [0.2, 0.2, 0.1]
                self.map_size_x = 512
                self.map_size_y = 512
                self.map_resolution = 0.2
                self.num_classes = 5
            
            self.get_logger().info(f'Model loaded successfully')
            self.get_logger().info(f'Point cloud range: {self.point_cloud_range}')
            self.get_logger().info(f'Voxel size: {self.voxel_size}')
            self.get_logger().info(f'Map size: {self.map_size_x}x{self.map_size_y}')
            self.get_logger().info(f'Number of classes: {self.num_classes}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            import traceback
            traceback.print_exc()
            raise
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 订阅和发布
        self.pc_sub = self.create_subscription(
            PointCloud2,
            pointcloud_topic,
            self.pointcloud_callback,
            qos
        )
        
        # 发布costmap
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/bevnet/costmap',
            10
        )
        
        # 发布可视化
        self.viz_pub = self.create_publisher(
            Image,
            '/bevnet/visualization',
            10
        )
        
        # 发布原始语义地图（用于调试）
        self.semantic_pub = self.create_publisher(
            OccupancyGrid,
            '/bevnet/semantic_map',
            10
        )
        
        # 语义类别到代价的映射
        # 基于KITTI数据集的5类别系统：0-free, 1-terrain, 2-vegetation, 3-obstacle, 4-unknown
        self.semantic_to_cost = {
            0: 0,     # free space
            1: 30,    # terrain (low cost)
            2: 60,    # vegetation (medium cost)
            3: 100,   # obstacles (lethal)
            4: -1,    # unknown
        }
        
        # 可视化的颜色映射 (BGR格式)
        self.color_map = np.array([
            [0, 255, 0],      # 0: free (green)
            [0, 255, 255],    # 1: terrain (yellow)
            [255, 0, 0],      # 2: vegetation (blue)
            [0, 0, 255],      # 3: obstacles (red)
            [128, 128, 128],  # 4: unknown (gray)
        ], dtype=np.uint8)
        
        # 如果是循环模型，存储位姿信息
        if self.model_type == 'recurrent':
            self.last_pose = np.eye(4, dtype=np.float32)
        
        self.get_logger().info('BEVNet inference node initialized')
    
    def pointcloud_to_numpy(self, msg):
        """将PointCloud2消息转换为numpy数组"""
        # 使用sensor_msgs_py库解析点云
        points_list = []
        for point in point_cloud2.read_points(msg, field_names=("x", "y", "z", "intensity"), skip_nans=True):
            points_list.append([point[0], point[1], point[2], point[3] if len(point) > 3 else 0.0])
        
        if len(points_list) == 0:
            self.get_logger().warn("Received empty point cloud")
            return None
            
        points = np.array(points_list, dtype=np.float32)
        return points
    
    def pointcloud_callback(self, msg):
        """处理点云数据"""
        try:
            # 转换点云
            points = self.pointcloud_to_numpy(msg)
            if points is None:
                return
            
            # BEVNet推理
            if self.model_type == 'single':
                # 单帧推理
                bev_preds = self.model.predict(points)
            else:
                # 循环推理需要位姿信息
                # 这里简化处理，使用单位矩阵作为相对位姿
                # 实际使用时应该从TF或其他来源获取真实位姿
                pose = np.eye(4, dtype=np.float32)
                bev_preds = self.model.predict(points, pose)
            
            # 将预测结果转换为numpy数组
            if isinstance(bev_preds, torch.Tensor):
                bev_semantic = bev_preds.argmax(dim=0).cpu().numpy()
                bev_probs = torch.softmax(bev_preds, dim=0).cpu().numpy()
            else:
                # 已经是numpy数组
                bev_semantic = np.argmax(bev_preds, axis=0)
                bev_probs = bev_preds
            
            # 确保地图大小正确
            if bev_semantic.shape != (self.map_size_y, self.map_size_x):
                self.get_logger().warn(f"BEV map size mismatch: expected {(self.map_size_y, self.map_size_x)}, "
                                     f"got {bev_semantic.shape}. Resizing...")
                bev_semantic = cv2.resize(bev_semantic.astype(np.uint8), 
                                        (self.map_size_x, self.map_size_y), 
                                        interpolation=cv2.INTER_NEAREST)
            
            # 发布语义地图
            self.publish_semantic_map(bev_semantic, msg.header)
            
            # 发布costmap
            self.publish_costmap(bev_semantic, msg.header)
            
            # 可视化
            if self.get_parameter('visualize').value:
                self.publish_visualization(bev_semantic, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'Error in pointcloud callback: {e}')
            import traceback
            traceback.print_exc()
    
    def publish_semantic_map(self, semantic_map, header):
        """发布原始语义地图"""
        msg = OccupancyGrid()
        msg.header = header
        msg.header.frame_id = 'base_link'
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_size_x
        msg.info.height = self.map_size_y
        msg.info.origin.position.x = self.point_cloud_range[0]
        msg.info.origin.position.y = self.point_cloud_range[1]
        msg.info.origin.orientation.w = 1.0
        
        # 将语义标签转换为0-100的值以便可视化
        semantic_vis = (semantic_map.astype(np.float32) / (self.num_classes - 1) * 100).astype(np.int8)
        
        # 翻转y轴以匹配ROS坐标系
        semantic_vis_flipped = np.flipud(semantic_vis)
        msg.data = semantic_vis_flipped.flatten().tolist()
        
        self.semantic_pub.publish(msg)
    
    def publish_costmap(self, semantic_map, header):
        """发布Nav2兼容的costmap"""
        # 创建costmap
        costmap = np.zeros_like(semantic_map, dtype=np.int8)
        
        for sem_class, cost in self.semantic_to_cost.items():
            if sem_class < self.num_classes:
                mask = semantic_map == sem_class
                costmap[mask] = cost
        
        # 创建OccupancyGrid消息
        msg = OccupancyGrid()
        msg.header = header
        msg.header.frame_id = 'base_link'
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_size_x
        msg.info.height = self.map_size_y
        msg.info.origin.position.x = self.point_cloud_range[0]
        msg.info.origin.position.y = self.point_cloud_range[1]
        msg.info.origin.orientation.w = 1.0
        
        # 翻转y轴以匹配ROS坐标系
        costmap_flipped = np.flipud(costmap)
        msg.data = costmap_flipped.flatten().tolist()
        
        self.costmap_pub.publish(msg)
    
    def publish_visualization(self, semantic_map, header):
        """发布可视化图像"""
        # 创建彩色语义图
        h, w = semantic_map.shape
        color_map = np.zeros((h, w, 3), dtype=np.uint8)
        
        for class_id in range(self.num_classes):
            if class_id < len(self.color_map):
                mask = semantic_map == class_id
                color_map[mask] = self.color_map[class_id]
        
        # 翻转图像以匹配ROS坐标系
        color_map_flipped = np.flipud(color_map)
        
        # 转换为ROS图像消息
        img_msg = self.bridge.cv2_to_imgmsg(color_map_flipped, encoding='bgr8')
        img_msg.header = header
        
        self.viz_pub.publish(img_msg)

def main(args=None):
    rclpy.init(args=args)
    node = BEVNetInferenceNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
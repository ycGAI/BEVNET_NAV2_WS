#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from nav_msgs.msg import OccupancyGrid
from bevnet_nav2_msgs.msg import BEVMap, SemanticCostmap
import numpy as np
import torch
import ros2_numpy
from bevnet.inference import BEVNetSingle
import cv2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

class BEVNetInferenceNode(Node):
    def __init__(self):
        super().__init__('bevnet_inference_node')
        
        # 声明参数
        self.declare_parameter('model_path', '')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('visualize', True)
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        device = self.get_parameter('device').value
        
        # 加载模型
        self.get_logger().info(f'Loading BEVNet model from: {model_path}')
        self.model = BEVNetSingle(model_path, device=device)
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅和发布
        self.pc_sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points',
            self.pointcloud_callback,
            qos
        )
        
        self.bevmap_pub = self.create_publisher(
            BEVMap,
            '/bevnet/semantic_map',
            10
        )
        
        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            '/bevnet/costmap',
            10
        )
        
        self.viz_pub = self.create_publisher(
            sensor_msgs.msg.Image,
            '/bevnet/visualization',
            10
        )
        
        # BEV参数
        self.map_size = 512
        self.map_resolution = 0.2
        self.map_range = 51.2
        
        self.get_logger().info('BEVNet inference node initialized')
    
    def pointcloud_callback(self, msg):
        """处理点云数据"""
        try:
            # 转换点云
            pc_array = ros2_numpy.point_cloud2.pointcloud2_to_array(msg)
            points = np.stack([
                pc_array['x'],
                pc_array['y'], 
                pc_array['z'],
                pc_array['intensity']
            ], axis=-1)
            
            # BEVNet推理
            with torch.no_grad():
                bev_logits = self.model.predict(points)[0]  # [C, H, W]
                bev_semantic = torch.argmax(bev_logits, dim=0).cpu().numpy()
                bev_probs = torch.softmax(bev_logits, dim=0).cpu().numpy()
            
            # 发布语义地图
            self.publish_bev_map(bev_semantic, bev_probs, msg.header)
            
            # 发布costmap
            self.publish_costmap(bev_semantic, bev_probs, msg.header)
            
            # 可视化
            if self.get_parameter('visualize').value:
                self.publish_visualization(bev_semantic, msg.header)
                
        except Exception as e:
            self.get_logger().error(f'Error in pointcloud callback: {e}')
    
    def publish_bev_map(self, semantic_map, probs, header):
        """发布BEV语义地图"""
        msg = BEVMap()
        msg.header = header
        msg.header.frame_id = 'base_link'
        
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = -self.map_range / 2
        msg.info.origin.position.y = -self.map_range / 2
        
        # 语义数据
        msg.semantic_map = semantic_map.flatten().tolist()
        msg.num_classes = self.model.g.num_class
        
        # 概率数据（可选）
        msg.class_probabilities = probs.flatten().tolist()
        
        self.bevmap_pub.publish(msg)
    
    def publish_costmap(self, semantic_map, probs, header):
        """发布Nav2兼容的costmap"""
        # 语义到代价的映射
        semantic_to_cost = {
            0: 0,    # free
            1: 20,   # low cost  
            2: 50,   # medium cost
            3: 100,  # obstacle
            4: -1    # unknown
        }
        
        # 创建costmap
        costmap = np.zeros_like(semantic_map, dtype=np.int8)
        for sem_class, cost in semantic_to_cost.items():
            mask = semantic_map == sem_class
            costmap[mask] = cost
        
        # 创建OccupancyGrid消息
        msg = OccupancyGrid()
        msg.header = header
        msg.header.frame_id = 'base_link'
        msg.info.resolution = self.map_resolution
        msg.info.width = self.map_size
        msg.info.height = self.map_size
        msg.info.origin.position.x = -self.map_range / 2
        msg.info.origin.position.y = -self.map_range / 2
        msg.info.origin.orientation.w = 1.0
        
        msg.data = costmap.flatten().tolist()
        
        self.costmap_pub.publish(msg)
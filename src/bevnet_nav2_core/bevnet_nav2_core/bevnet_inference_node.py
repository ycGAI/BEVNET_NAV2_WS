#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/workspace/bevnet')

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, Image
from nav_msgs.msg import OccupancyGrid, MapMetaData
from bevnet_nav2_msgs.msg import BEVMap  # 使用自定义消息
from std_msgs.msg import Header
import numpy as np
import torch
import struct
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# BEVNet imports
from bevnet.inference import BEVNetSingle, BEVNetRecurrent

# 尝试导入cv_bridge
try:
    import cv2
    from cv_bridge import CvBridge
    HAS_CV_BRIDGE = True
except ImportError:
    print("Warning: cv_bridge not available")
    HAS_CV_BRIDGE = False

class BEVNetInferenceNode(Node):
    def __init__(self):
        super().__init__('bevnet_inference_node')
        
        # 声明参数
        self.declare_parameter('model_path', '/workspace/bevnet_nav2_ws/models/best.pth.34')
        self.declare_parameter('model_type', 'single')
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('pointcloud_topic', '/velodyne_points')
        self.declare_parameter('visualize', True)
        self.declare_parameter('publish_semantic', True)
        self.declare_parameter('publish_costmap', True)
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        model_type = self.get_parameter('model_type').value
        device = self.get_parameter('device').value
        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        self.publish_semantic = self.get_parameter('publish_semantic').value
        self.publish_costmap_flag = self.get_parameter('publish_costmap').value
        
        self.get_logger().info(f'Loading BEVNet model from: {model_path}')
        self.get_logger().info(f'Model type: {model_type}')
        self.get_logger().info(f'Device: {device}')
        
        try:
            if model_type == 'single':
                self.model = BEVNetSingle(model_path, device=device)
            else:
                self.model = BEVNetRecurrent(model_path, device=device)
            self.get_logger().info('Model loaded successfully!')
        except Exception as e:
            self.get_logger().error(f'Failed to load model: {e}')
            raise
        
        # 设置参数
        self.map_size = 407
        self.map_resolution = 0.2  # meters per pixel
        self.num_classes = 5
        self.point_cloud_range = [-40.7, -40.7, -2, 40.7, 40.7, 1.0]  # 调整为对称范围
        
        # 语义类别定义
        self.class_names = [
            "free_space",    # 0: 可通行区域
            "road",          # 1: 道路
            "sidewalk",      # 2: 人行道
            "obstacle",      # 3: 障碍物
            "unknown"        # 4: 未知区域
        ]
        
        # 语义到代价映射 (符合Nav2标准: 0-254可通行程度, 255未知)
        self.semantic_to_cost = {
            0: 0,      # free_space -> 完全自由
            1: 10,     # road -> 轻微代价（优先道路）
            2: 30,     # sidewalk -> 低代价
            3: 254,    # obstacle -> 致命障碍
            4: 255,    # unknown -> 未知
        }
        
        # 颜色映射（用于可视化）
        self.color_map = np.array([
            [0, 255, 0],      # 0: free (green)
            [128, 128, 255],  # 1: road (light blue)
            [255, 255, 0],    # 2: sidewalk (yellow)
            [255, 0, 0],      # 3: obstacles (red)
            [128, 128, 128],  # 4: unknown (gray)
        ], dtype=np.uint8)
        
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
        # 1. 导航用的代价地图（OccupancyGrid格式，Nav2直接使用）
        if self.publish_costmap_flag:
            self.costmap_pub = self.create_publisher(
                OccupancyGrid,
                '/bevnet/costmap',
                10
            )
        
        # 2. 语义地图（BEVMap格式，包含语义信息）
        if self.publish_semantic:
            self.semantic_pub = self.create_publisher(
                BEVMap,
                '/bevnet/semantic_map',
                10
            )
        
        # 3. 可视化
        if HAS_CV_BRIDGE and self.get_parameter('visualize').value:
            self.bridge = CvBridge()
            self.viz_pub = self.create_publisher(
                Image,
                '/bevnet/visualization',
                10
            )
        else:
            self.viz_pub = None
            
        self.get_logger().info(f'BEVNet inference node initialized')
        self.get_logger().info(f'Subscribing to: {pointcloud_topic}')
        self.get_logger().info(f'Publishing costmap: {self.publish_costmap_flag}')
        self.get_logger().info(f'Publishing semantic: {self.publish_semantic}')
        
        # 统计信息
        self.frame_count = 0
        self.last_time = self.get_clock().now()
    
    def pointcloud_to_numpy(self, msg):
        """将PointCloud2消息转换为numpy数组"""
        points = []
        
        # 获取字段偏移
        x_offset = y_offset = z_offset = intensity_offset = None
        
        for field in msg.fields:
            if field.name == 'x':
                x_offset = field.offset
            elif field.name == 'y':
                y_offset = field.offset
            elif field.name == 'z':
                z_offset = field.offset
            elif field.name == 'intensity':
                intensity_offset = field.offset
        
        if intensity_offset is None:
            intensity_offset = 12  # 默认偏移
        
        # 解析数据
        point_step = msg.point_step
        data = msg.data
        
        for i in range(0, len(data), point_step):
            x = struct.unpack('f', data[i+x_offset:i+x_offset+4])[0]
            y = struct.unpack('f', data[i+y_offset:i+y_offset+4])[0]
            z = struct.unpack('f', data[i+z_offset:i+z_offset+4])[0]
            
            if i+intensity_offset+4 <= len(data):
                intensity = struct.unpack('f', data[i+intensity_offset:i+intensity_offset+4])[0]
            else:
                intensity = 0.0
                
            # 过滤无效点和范围外的点
            if not (np.isnan(x) or np.isnan(y) or np.isnan(z)):
                if (self.point_cloud_range[0] <= x <= self.point_cloud_range[3] and
                    self.point_cloud_range[1] <= y <= self.point_cloud_range[4] and
                    self.point_cloud_range[2] <= z <= self.point_cloud_range[5]):
                    points.append([x, y, z, intensity])
        
        if len(points) == 0:
            return None
            
        return np.array(points, dtype=np.float32)
    
    def pointcloud_callback(self, msg):
        """处理点云数据"""
        try:
            # 转换点云
            points = self.pointcloud_to_numpy(msg)
            if points is None or len(points) < 100:
                self.get_logger().warn(f"Too few valid points: {len(points) if points is not None else 0}")
                return
            
            # BEVNet推理
            with torch.no_grad():
                output = self.model.predict(points)
                
                # 处理输出
                if torch.is_tensor(output):
                    output = output.cpu().numpy()
                
                # output shape: [1, 5, 407, 407]
                if len(output.shape) == 4:
                    output = output[0]  # [5, 407, 407]
                
                # 获取语义分割结果和概率
                semantic_map = np.argmax(output, axis=0)  # [407, 407]
                class_probabilities = np.transpose(output, (1, 2, 0))  # [407, 407, 5]
            
            # 发布不同格式的地图
            if self.publish_semantic:
                self.publish_semantic_map(semantic_map, class_probabilities, msg.header)
            
            if self.publish_costmap_flag:
                self.publish_costmap(semantic_map, msg.header)
            
            if self.viz_pub is not None:
                self.publish_visualization(semantic_map, msg.header)
            
            # 更新统计
            self.update_statistics()
                
        except Exception as e:
            self.get_logger().error(f'Error in pointcloud callback: {e}')
            import traceback
            traceback.print_exc()
    
    def publish_semantic_map(self, semantic_map, class_probabilities, header):
        """发布语义地图 - 使用BEVMap消息类型"""
        msg = BEVMap()
        msg.header = header
        msg.header.frame_id = 'base_link'
        
        # 设置地图元数据
        msg.info = MapMetaData()
        msg.info.resolution = self.map_resolution
        msg.info.width = semantic_map.shape[1]
        msg.info.height = semantic_map.shape[0]
        msg.info.origin.position.x = self.point_cloud_range[0]
        msg.info.origin.position.y = self.point_cloud_range[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # 语义数据（翻转以匹配ROS坐标系）
        semantic_flipped = np.flipud(semantic_map)
        msg.semantic_map = semantic_flipped.flatten().astype(np.uint8).tolist()
        msg.num_classes = np.uint8(self.num_classes)
        
        # 类别名称
        msg.class_names = self.class_names
        
        # 可选：添加类别概率（用于不确定性分析）
        if class_probabilities is not None:
            probs_flipped = np.flipud(class_probabilities)
            msg.class_probabilities = probs_flipped.flatten().astype(np.float32).tolist()
        
        self.semantic_pub.publish(msg)
        self.get_logger().debug(f"Published semantic map with shape {semantic_map.shape}")
    
    def publish_costmap(self, semantic_map, header):
        """发布Nav2兼容的代价地图 - OccupancyGrid格式"""
        # 将语义转换为导航代价
        costmap = np.full_like(semantic_map, 255, dtype=np.int8)  # 默认未知
        
        for sem_class, cost in self.semantic_to_cost.items():
            if sem_class < self.num_classes:
                mask = semantic_map == sem_class
                # Nav2使用signed int8: -1表示未知，0-100表示占用概率
                if cost == 255:
                    costmap[mask] = -1  # 未知
                elif cost >= 254:
                    costmap[mask] = 100  # 完全占用
                else:
                    # 线性映射到0-100
                    costmap[mask] = int(cost * 100 / 254)
        
        msg = OccupancyGrid()
        msg.header = header
        msg.header.frame_id = 'base_link'
        msg.info.resolution = self.map_resolution
        msg.info.width = costmap.shape[1]
        msg.info.height = costmap.shape[0]
        msg.info.origin.position.x = self.point_cloud_range[0]
        msg.info.origin.position.y = self.point_cloud_range[1]
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        
        # 翻转以匹配ROS坐标系
        msg.data = np.flipud(costmap).flatten().tolist()
        self.costmap_pub.publish(msg)
        self.get_logger().debug(f"Published costmap with shape {costmap.shape}")
    
    def publish_visualization(self, semantic_map, header):
        """发布可视化图像"""
        if not HAS_CV_BRIDGE or self.viz_pub is None:
            return
            
        # 创建彩色图像
        h, w = semantic_map.shape
        color_image = np.zeros((h, w, 3), dtype=np.uint8)
        
        for class_id in range(self.num_classes):
            mask = semantic_map == class_id
            color_image[mask] = self.color_map[class_id]
        
        # 添加网格线以便调试
        grid_step = int(10 / self.map_resolution)  # 每10米一条线
        color_image[::grid_step, :] = [64, 64, 64]  # 水平线
        color_image[:, ::grid_step] = [64, 64, 64]  # 垂直线
        
        # 翻转以匹配ROS坐标系
        color_image_flipped = np.flipud(color_image)
        
        # 转换为ROS消息
        img_msg = self.bridge.cv2_to_imgmsg(color_image_flipped, encoding='bgr8')
        img_msg.header = header
        self.viz_pub.publish(img_msg)
    
    def update_statistics(self):
        """更新和打印统计信息"""
        self.frame_count += 1
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 1.0:  # 每秒打印一次
            fps = self.frame_count / dt
            self.get_logger().info(
                f'Processing at {fps:.1f} FPS | '
                f'Map size: {self.map_size}x{self.map_size} | '
                f'Resolution: {self.map_resolution}m/pixel'
            )
            self.frame_count = 0
            self.last_time = current_time

def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = BEVNetInferenceNode()
        rclpy.spin(node)
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
# export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH
# cd /bevnet_nav2_ws
# python3 src/bevnet_nav2_core/bevnet_nav2_core/bevnet_inference_node.py
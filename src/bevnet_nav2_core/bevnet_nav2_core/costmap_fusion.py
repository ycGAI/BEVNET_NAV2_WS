#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from bevnet_nav2_msgs.msg import BEVMap, SemanticCostmap
import numpy as np
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
import cv2

class CostmapFusion(Node):
    def __init__(self):
        super().__init__('costmap_fusion')
        
        # 参数
        self.declare_parameter('fusion_method', 'weighted_average')
        self.declare_parameter('bevnet_weight', 0.7)
        self.declare_parameter('publish_rate', 10.0)
        
        self.fusion_method = self.get_parameter('fusion_method').value
        self.bevnet_weight = self.get_parameter('bevnet_weight').value
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # 订阅
        self.bevnet_sub = self.create_subscription(
            OccupancyGrid,
            '/bevnet/costmap',
            self.bevnet_callback,
            qos
        )
        
        self.local_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/local_costmap/costmap',
            self.local_costmap_callback,
            qos
        )
        
        self.global_costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/global_costmap/costmap',
            self.global_costmap_callback,
            qos
        )
        
        # 发布
        self.fused_local_pub = self.create_publisher(
            OccupancyGrid,
            '/fused_local_costmap',
            10
        )
        
        self.fused_global_pub = self.create_publisher(
            OccupancyGrid,
            '/fused_global_costmap',
            10
        )
        
        # 数据缓存
        self.bevnet_costmap = None
        self.local_costmap = None
        self.global_costmap = None
        
        # 定时器
        self.timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value,
            self.fusion_callback
        )
        
        self.get_logger().info('Costmap fusion node initialized')
    
    def bevnet_callback(self, msg):
        self.bevnet_costmap = msg
    
    def local_costmap_callback(self, msg):
        self.local_costmap = msg
    
    def global_costmap_callback(self, msg):
        self.global_costmap = msg
    
    def fusion_callback(self):
        """融合costmap"""
        # 融合局部costmap
        if self.bevnet_costmap and self.local_costmap:
            fused_local = self.fuse_costmaps(
                self.bevnet_costmap,
                self.local_costmap,
                is_local=True
            )
            if fused_local:
                self.fused_local_pub.publish(fused_local)
        
        # 融合全局costmap
        if self.bevnet_costmap and self.global_costmap:
            fused_global = self.fuse_costmaps(
                self.bevnet_costmap,
                self.global_costmap,
                is_local=False
            )
            if fused_global:
                self.fused_global_pub.publish(fused_global)
    
    def fuse_costmaps(self, bevnet_map, nav2_map, is_local=True):
        """融合两个costmap"""
        try:
            # 检查分辨率是否匹配
            if abs(bevnet_map.info.resolution - nav2_map.info.resolution) > 0.001:
                self.get_logger().warn('Resolution mismatch, resampling BEVNet costmap')
                bevnet_map = self.resample_costmap(bevnet_map, nav2_map.info.resolution)
            
            # 转换到同一坐标系
            bevnet_array = self.costmap_to_array(bevnet_map)
            nav2_array = self.costmap_to_array(nav2_map)
            
            # 对齐地图
            aligned_bevnet, aligned_nav2 = self.align_costmaps(
                bevnet_array, bevnet_map.info,
                nav2_array, nav2_map.info
            )
            
            # 融合
            if self.fusion_method == 'weighted_average':
                fused_array = self.weighted_average_fusion(
                    aligned_bevnet, aligned_nav2
                )
            elif self.fusion_method == 'maximum':
                fused_array = np.maximum(aligned_bevnet, aligned_nav2)
            elif self.fusion_method == 'bevnet_priority':
                fused_array = self.bevnet_priority_fusion(
                    aligned_bevnet, aligned_nav2
                )
            else:
                fused_array = aligned_bevnet
            
            # 创建输出消息
            fused_msg = OccupancyGrid()
            fused_msg.header = nav2_map.header
            fused_msg.info = nav2_map.info
            fused_msg.data = fused_array.flatten().astype(np.int8).tolist()
            
            return fused_msg
            
        except Exception as e:
            self.get_logger().error(f'Error in fusion: {e}')
            return None
    
    def costmap_to_array(self, costmap):
        """将costmap消息转换为numpy数组"""
        data = np.array(costmap.data, dtype=np.int8)
        return data.reshape((costmap.info.height, costmap.info.width))
    
    def resample_costmap(self, costmap, target_resolution):
        """重采样costmap到目标分辨率"""
        # 计算缩放因子
        scale = costmap.info.resolution / target_resolution
        
        # 转换为数组
        array = self.costmap_to_array(costmap)
        
        # 重采样
        new_height = int(costmap.info.height * scale)
        new_width = int(costmap.info.width * scale)
        resampled = cv2.resize(
            array,
            (new_width, new_height),
            interpolation=cv2.INTER_NEAREST
        )
        
        # 创建新的costmap消息
        new_costmap = OccupancyGrid()
        new_costmap.header = costmap.header
        new_costmap.info = costmap.info
        new_costmap.info.resolution = target_resolution
        new_costmap.info.width = new_width
        new_costmap.info.height = new_height
        new_costmap.data = resampled.flatten().astype(np.int8).tolist()
        
        return new_costmap
    
    def align_costmaps(self, array1, info1, array2, info2):
        """对齐两个costmap数组"""
        # 计算重叠区域
        x1_min = info1.origin.position.x
        y1_min = info1.origin.position.y
        x1_max = x1_min + info1.width * info1.resolution
        y1_max = y1_min + info1.height * info1.resolution
        
        x2_min = info2.origin.position.x
        y2_min = info2.origin.position.y
        x2_max = x2_min + info2.width * info2.resolution
        y2_max = y2_min + info2.height * info2.resolution
        
        # 重叠区域
        x_min = max(x1_min, x2_min)
        y_min = max(y1_min, y2_min)
        x_max = min(x1_max, x2_max)
        y_max = min(y1_max, y2_max)
        
        if x_max <= x_min or y_max <= y_min:
            # 没有重叠
            return array1, array2
        
        # 在各自数组中的索引
        i1_min = int((x_min - x1_min) / info1.resolution)
        j1_min = int((y_min - y1_min) / info1.resolution)
        i1_max = int((x_max - x1_min) / info1.resolution)
        j1_max = int((y_max - y1_min) / info1.resolution)
        
        i2_min = int((x_min - x2_min) / info2.resolution)
        j2_min = int((y_min - y2_min) / info2.resolution)
        i2_max = int((x_max - x2_min) / info2.resolution)
        j2_max = int((y_max - y2_min) / info2.resolution)
        
        # 提取重叠区域
        aligned1 = array1[j1_min:j1_max, i1_min:i1_max]
        aligned2 = array2[j2_min:j2_max, i2_min:i2_max]
        
        # 确保大小一致
        min_height = min(aligned1.shape[0], aligned2.shape[0])
        min_width = min(aligned1.shape[1], aligned2.shape[1])
        
        return aligned1[:min_height, :min_width], aligned2[:min_height, :min_width]
    
    def weighted_average_fusion(self, bevnet_array, nav2_array):
        """加权平均融合"""
        # 处理未知值（-1）
        bevnet_mask = bevnet_array >= 0
        nav2_mask = nav2_array >= 0
        
        # 初始化融合结果
        fused = np.full_like(bevnet_array, -1)
        
        # 两者都有值的区域：加权平均
        both_valid = bevnet_mask & nav2_mask
        fused[both_valid] = (
            self.bevnet_weight * bevnet_array[both_valid] +
            (1 - self.bevnet_weight) * nav2_array[both_valid]
        ).astype(np.int8)
        
        # 只有BEVNet有值的区域
        bevnet_only = bevnet_mask & ~nav2_mask
        fused[bevnet_only] = bevnet_array[bevnet_only]
        
        # 只有Nav2有值的区域
        nav2_only = ~bevnet_mask & nav2_mask
        fused[nav2_only] = nav2_array[nav2_only]
        
        return fused
    
    def bevnet_priority_fusion(self, bevnet_array, nav2_array):
        """BEVNet优先级融合"""
        # BEVNet有效值优先
        bevnet_mask = bevnet_array >= 0
        fused = nav2_array.copy()
        fused[bevnet_mask] = bevnet_array[bevnet_mask]
        
        return fused

def main(args=None):
    rclpy.init(args=args)
    node = CostmapFusion()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
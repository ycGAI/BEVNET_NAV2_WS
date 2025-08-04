#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import Image
import numpy as np
import cv2
import os

class ResultSaver(Node):
    def __init__(self):
        super().__init__('result_saver')
        
        # 创建保存目录
        os.makedirs('bevnet_results', exist_ok=True)
        
        # 订阅
        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            '/bevnet/costmap',
            self.costmap_callback,
            1
        )
        
        self.semantic_sub = self.create_subscription(
            OccupancyGrid,
            '/bevnet/semantic_map', 
            self.semantic_callback,
            1
        )
        
        self.count = 0
        self.max_saves = 5
        
        # 颜色映射
        self.color_map = np.array([
            [0, 255, 0],      # 0: free (green)
            [255, 255, 0],    # 1: terrain (yellow)  
            [0, 0, 255],      # 2: vegetation (blue)
            [255, 0, 0],      # 3: obstacles (red)
            [128, 128, 128],  # 4: unknown (gray)
        ], dtype=np.uint8)
        
        self.get_logger().info('Result saver started, will save 5 frames...')
        
    def costmap_callback(self, msg):
        if self.count >= self.max_saves:
            return
            
        self.get_logger().info(f'Saving costmap {self.count}: {msg.info.width}x{msg.info.height}')
        
        # 转换为numpy数组
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        
        # 可视化costmap
        vis = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)
        vis[data == 0] = [0, 255, 0]       # Free - green
        vis[data == 100] = [255, 0, 0]     # Lethal - red
        vis[data == -1] = [128, 128, 128]  # Unknown - gray
        vis[(data > 0) & (data < 100)] = [255, 255, 0]  # Cost - yellow
        
        filename = f'bevnet_results/costmap_{self.count:04d}.png'
        cv2.imwrite(filename, vis)
        self.get_logger().info(f'Saved {filename}')
        
        self.count += 1
        if self.count >= self.max_saves:
            self.get_logger().info('Saved 5 frames, stopping...')
            rclpy.shutdown()
        
    def semantic_callback(self, msg):
        if self.count >= self.max_saves:
            return
            
        # 转换为numpy数组  
        data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        
        # 转换回语义类别 (0-100 -> 0-4)
        semantic = (data * (5-1) / 100.0).astype(np.uint8)
        
        # 创建彩色图
        vis = np.zeros((msg.info.height, msg.info.width, 3), dtype=np.uint8)
        for i in range(5):
            vis[semantic == i] = self.color_map[i]
        
        filename = f'bevnet_results/semantic_{self.count:04d}.png'
        cv2.imwrite(filename, vis)

def main():
    rclpy.init()
    node = ResultSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

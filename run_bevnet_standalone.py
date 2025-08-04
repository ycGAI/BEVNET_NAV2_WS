#!/usr/bin/env python3
"""
BEVNet，模拟ROS2功能
'EOF'
"""
import sys
import os
sys.path.insert(0, '/workspace/bevnet')

import numpy as np
import torch
import time
import argparse
from bevnet.inference import BEVNetSingle
import cv2

class StandaloneBEVNet:
    def __init__(self, model_path, visualize=True):
        print("Initializing BEVNet in standalone mode...")
        
        # 加载模型
        self.model = BEVNetSingle(model_path, device='cuda' if torch.cuda.is_available() else 'cpu')
        print("Model loaded successfully!")
        
        self.visualize = visualize
        self.frame_count = 0
        
        # 参数
        self.map_size = 407
        self.map_resolution = 0.2
        self.num_classes = 5
        
        # 颜色映射
        self.color_map = np.array([
            [0, 255, 0],      # 0: free (green)
            [0, 255, 255],    # 1: terrain (yellow)
            [0, 0, 255],      # 2: vegetation (blue)
            [255, 0, 0],      # 3: obstacles (red)
            [128, 128, 128],  # 4: unknown (gray)
        ], dtype=np.uint8)
        
    def generate_test_pointcloud(self):
        """生成测试点云数据"""
        num_points = 50000
        points = np.zeros((num_points, 4), dtype=np.float32)
        
        # 生成环形点云（模拟激光雷达）
        angles = np.random.uniform(0, 2*np.pi, num_points)
        distances = np.random.uniform(0.5, 30, num_points)
        heights = np.random.uniform(-2, 0.5, num_points)
        
        points[:, 0] = distances * np.cos(angles)
        points[:, 1] = distances * np.sin(angles)
        points[:, 2] = heights
        points[:, 3] = np.random.uniform(0, 1, num_points)
        
        # 添加一些结构（模拟建筑物）
        # 在某些区域增加点的密度
        for i in range(5):
            cx, cy = np.random.uniform(-20, 20, 2)
            mask = ((points[:, 0] - cx)**2 + (points[:, 1] - cy)**2) < 25
            points[mask, 2] = np.random.uniform(0, 2, np.sum(mask))
            
        return points
    
    def process_pointcloud(self, points):
        """处理点云"""
        start_time = time.time()
        
        # 运行推理
        with torch.no_grad():
            output = self.model.predict(points)
            
            if torch.is_tensor(output):
                output = output.cpu().numpy()
            
            if len(output.shape) == 4:
                output = output[0]
            
            semantic_map = np.argmax(output, axis=0)
        
        process_time = time.time() - start_time
        
        return semantic_map, process_time
    
    def visualize_result(self, semantic_map):
        """可视"""
        h, w = semantic_map.shape
        color_image = np.zeros((h, w, 3), dtype=np.uint8)
        
        for class_id in range(self.num_classes):
            mask = semantic_map == class_id
            color_image[mask] = self.color_map[class_id]
        
        # 添加文字信息
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(color_image, f'Frame: {self.frame_count}', (10, 30), 
                   font, 1, (255, 255, 255), 2)
        
        # 显示类别统计
        for i in range(self.num_classes):
            count = np.sum(semantic_map == i)
            percentage = count / semantic_map.size * 100
            text = f'Class {i}: {percentage:.1f}%'
            cv2.putText(color_image, text, (10, 60 + i*30), 
                       font, 0.6, self.color_map[i].tolist(), 2)
        
        return color_image
    
    def run_continuous(self, fps=10):
        """连续运行模式"""
        print(f"Running in continuous mode at {fps} FPS...")
        print("Press 'q' to quit, 's' to save image")
        
        cv2.namedWindow('BEVNet Semantic Map', cv2.WINDOW_NORMAL)
        
        interval = 1.0 / fps
        
        while True:
            # 生成或读取点云
            points = self.generate_test_pointcloud()
            
            # 处理
            semantic_map, process_time = self.process_pointcloud(points)
            
            # 可视化
            if self.visualize:
                color_image = self.visualize_result(semantic_map)
                cv2.imshow('BEVNet Semantic Map', color_image)
                
                key = cv2.waitKey(int(interval * 1000)) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    filename = f'bevnet_output_{self.frame_count}.png'
                    cv2.imwrite(filename, color_image)
                    print(f"Saved {filename}")
            
            # 'EOF'
            self.frame_count += 1
            if self.frame_count % fps == 0:
                print(f"Frame {self.frame_count}: Processing time = {process_time*1000:.1f}ms")
            
            # 控制帧率
            time.sleep(max(0, interval - process_time))
        
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description='BEVNet Standalone Runner')
    parser.add_argument('--model_path', type=str, 
                       default='/workspace/bevnet_nav2_ws/models/best.pth.34',
                       help='Path to model file')
    parser.add_argument('--fps', type=int, default=10,
                       help='Target FPS for continuous mode')
    parser.add_argument('--no-viz', action='store_true',
                       help='Disable visualization')
    
    args = parser.parse_args()
    
    # 创建处理器
    processor = StandaloneBEVNet(args.model_path, visualize=not args.no_viz)
    
    # 运行
    processor.run_continuous(fps=args.fps)

if __name__ == '__main__':
    main()

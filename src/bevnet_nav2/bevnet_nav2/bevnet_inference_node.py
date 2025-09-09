#!/usr/bin/env python3
import sys
import os
sys.path.insert(0, '/workspace/bevnet')

import numpy as np
import torch
import yaml
from bevnet.inference import BEVNetSingle, BEVNetRecurrent

def main():
    print("BEVNet Inference Node")
    print("=" * 50)
    
    # 参数
    model_path = None
    model_type = 'single'
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    
#    # 简单的命令行参数

    import argparse
    parser = argparse.ArgumentParser(description='BEVNet Inference Node')
    parser.add_argument('--model_path', type=str, required=True, help='Path to model file')
    parser.add_argument('--model_type', type=str, default='single', choices=['single', 'recurrent'])
    parser.add_argument('--device', type=str, default='cuda', choices=['cuda', 'cpu'])
    
    args = parser.parse_args()
    
    print(f"Model path: {args.model_path}")
    print(f"Model type: {args.model_type}")
    print(f"Device: {args.device}")
    
    # 检查模型文件
    if not os.path.exists(args.model_path):
        print(f"Error: Model file not found: {args.model_path}")
        return 1
    
    # 加载模
    try:
        if args.model_type == 'single':
            model = BEVNetSingle(args.model_path, device=args.device)
        else:
            model = BEVNetRecurrent(args.model_path, device=args.device)
        print("Model loaded successfully!")
    except Exception as e:
        print(f"Error loading model: {e}")
        return 1
    
    # bevnet_inference_node.py bevnet_inference_node.py 
    print("\nTesting inference with random point cloud...")
    points = np.random.randn(10000, 4).astype(np.float32)
    points[:, :3] *= 20  # scale coordinates
    points[:, 3] = np.abs(points[:, 3])  # positive intensity
    
    try:
        output = model.predict(points)
        print(f"Inference successful! Output shape: {output.shape}")
    except Exception as e:
        print(f"Inference error: {e}")
        return 1
    
    print("\nBEVNet node ready for ROS integration!")
    return 0

if __name__ == '__main__':
    sys.exit(main())

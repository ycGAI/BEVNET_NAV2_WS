#!/usr/bin/env python3
"""
KITTI Calibration Debug - 对比Tr变换的影响
"""

import os
import numpy as np
from numpy.linalg import inv

def parse_calibration(filename):
    """解析标定文件"""
    calib = {}
    with open(filename, 'r') as f:
        for line in f:
            if ':' not in line:
                continue
            key, content = line.strip().split(":")
            values = [float(v) for v in content.strip().split()]
            
            pose = np.zeros((4, 4))
            if len(values) >= 12:
                pose[0, 0:4] = values[0:4]
                pose[1, 0:4] = values[4:8]
                pose[2, 0:4] = values[8:12]
                pose[3, 3] = 1.0
                calib[key] = pose
    return calib

def test_poses(dataset_path, sequence='train'):
    """测试位姿变换"""
    seq_dir = os.path.join(dataset_path, 'sequences', sequence)
    poses_file = os.path.join(seq_dir, 'poses.txt')
    calib_file = os.path.join(seq_dir, 'calib.txt')
    
    # 解析标定
    calibration = parse_calibration(calib_file)
    
    print("="*60)
    print("标定信息:")
    print("="*60)
    
    if "Tr" in calibration:
        Tr = calibration["Tr"]
        print("Tr矩阵（激光雷达到相机）:")
        print(Tr)
        print()
        
        Tr_inv = inv(Tr)
        print("Tr_inv矩阵（相机到激光雷达）:")
        print(Tr_inv)
        print()
    
    # 读取前几个位姿
    print("="*60)
    print("位姿对比（前5个）:")
    print("="*60)
    
    with open(poses_file, 'r') as f:
        for i, line in enumerate(f):
            if i >= 5:
                break
                
            values = [float(v) for v in line.strip().split()]
            if len(values) == 12:
                # 原始位姿
                pose_original = np.zeros((4, 4))
                pose_original[0, 0:4] = values[0:4]
                pose_original[1, 0:4] = values[4:8]
                pose_original[2, 0:4] = values[8:12]
                pose_original[3, 3] = 1.0
                
                print(f"\n帧 {i}:")
                print(f"  原始位姿 (X={values[3]:.2f}, Y={values[7]:.2f}, Z={values[11]:.2f})")
                
                if "Tr" in calibration:
                    # 应用Tr变换
                    pose_transformed = np.matmul(Tr_inv, np.matmul(pose_original, Tr))
                    print(f"  变换后 (X={pose_transformed[0,3]:.2f}, Y={pose_transformed[1,3]:.2f}, Z={pose_transformed[2,3]:.2f})")
                    
                    # 显示差异
                    dx = pose_transformed[0,3] - values[3]
                    dy = pose_transformed[1,3] - values[7]
                    dz = pose_transformed[2,3] - values[11]
                    print(f"  差异: ΔX={dx:.2f}, ΔY={dy:.2f}, ΔZ={dz:.2f}")

if __name__ == '__main__':
    import argparse
    parser = argparse.ArgumentParser(description='Test KITTI calibration')
    parser.add_argument('dataset_path', help='Path to dataset')
    parser.add_argument('--sequence', default='train', help='Sequence name')
    
    args = parser.parse_args()
    test_poses(args.dataset_path, args.sequence)
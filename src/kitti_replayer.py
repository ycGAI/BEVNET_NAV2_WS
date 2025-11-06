#!/usr/bin/env python3
"""
KITTI格式数据播放器 - Z坐标归一化版本
将高度归零，便于在RViz2中查看
"""
import os
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import argparse

class KittiPlayerNormalized(Node):
    def __init__(self, kitti_dir, sequence='valid', rate=10.0, loop=True, normalize_z=True):
        super().__init__('kitti_player_normalized')
        
        self.kitti_dir = kitti_dir
        self.sequence = sequence
        self.rate = rate
        self.loop = loop
        self.normalize_z = normalize_z
        
        # 设置数据目录
        self.seq_dir = os.path.join(kitti_dir, 'sequences', sequence)
        if not os.path.exists(self.seq_dir):
            self.get_logger().error(f'Sequence directory not found: {self.seq_dir}')
            return
        
        # 数据目录
        self.velodyne_dir = os.path.join(self.seq_dir, 'velodyne')
        self.poses_file = os.path.join(self.seq_dir, 'poses.txt')
        
        # 检查目录
        if not os.path.exists(self.velodyne_dir):
            self.get_logger().error(f'Velodyne directory not found: {self.velodyne_dir}')
            return
        
        # 获取bin文件
        self.bin_files = sorted([f for f in os.listdir(self.velodyne_dir) if f.endswith('.bin')])
        if not self.bin_files:
            self.get_logger().error(f'No .bin files found')
            return
        
        self.get_logger().info(f'Found {len(self.bin_files)} point cloud files')
        
        # 加载位姿
        self.poses = self.load_poses()
        
        # QoS配置
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # 发布器
        self.pc_pub = self.create_publisher(PointCloud2, '/velodyne_points', qos)
        self.odom_pub = self.create_publisher(Odometry, '/odom', qos)
        self.pose_pub = self.create_publisher(PoseStamped, '/current_pose', qos)
        self.path_pub = self.create_publisher(Path, '/path', qos)
        
        # TF广播器
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # 路径
        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'
        
        # 播放状态
        self.current_index = 0
        self.playing = True
        
        # 定时器
        self.timer = self.create_timer(1.0 / self.rate, self.publish_frame)
        
        self.print_info()
    
    def print_info(self):
        """打印信息"""
        self.get_logger().info('='*60)
        self.get_logger().info('KITTI Player (Z-Normalized)')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Directory: {self.seq_dir}')
        self.get_logger().info(f'Point clouds: {len(self.bin_files)}')
        self.get_logger().info(f'Poses: {len(self.poses) if self.poses else 0}')
        self.get_logger().info(f'Rate: {self.rate} Hz')
        self.get_logger().info(f'Loop: {self.loop}')
        self.get_logger().info(f'Normalize Z: {self.normalize_z}')
        self.get_logger().info('='*60)
    
    def parse_calibration(self, filename):
        """完全按照BEVNet的方式解析标定文件"""
        calib = {}
        
        try:
            with open(filename, 'r') as calib_file:
                for line in calib_file:
                    if ':' not in line:
                        continue
                    key, content = line.strip().split(":")
                    values = [float(v) for v in content.strip().split()]
                    
                    # 创建4x4矩阵
                    pose = np.zeros((4, 4))
                    if len(values) >= 12:
                        pose[0, 0:4] = values[0:4]
                        pose[1, 0:4] = values[4:8]
                        pose[2, 0:4] = values[8:12]
                        pose[3, 3] = 1.0
                        
                        calib[key] = pose
        except Exception as e:
            self.get_logger().warn(f'Error parsing calibration: {e}')
            
        return calib
    
    def load_poses(self):
        """加载位姿数据 - 严格按照BEVNet的方式"""
        if not os.path.exists(self.poses_file):
            self.get_logger().warn(f'Poses file not found: {self.poses_file}')
            return None
        
        # 解析标定文件
        calib_file = os.path.join(os.path.dirname(self.poses_file), 'calib.txt')
        calibration = self.parse_calibration(calib_file)
        
        # 获取Tr矩阵
        if "Tr" in calibration:
            Tr = calibration["Tr"]
            Tr_inv = np.linalg.inv(Tr)
            self.get_logger().info('Found Tr matrix for lidar-camera transformation')
        else:
            self.get_logger().warn('No Tr matrix found in calibration, using identity')
            Tr = np.eye(4)
            Tr_inv = np.eye(4)
        
        # 读取poses文件
        poses = []
        with open(self.poses_file, 'r') as file:
            for line in file:
                values = [float(v) for v in line.strip().split()]
                
                if len(values) == 12:
                    # 构建4x4位姿矩阵
                    pose = np.zeros((4, 4))
                    pose[0, 0:4] = values[0:4]
                    pose[1, 0:4] = values[4:8]
                    pose[2, 0:4] = values[8:12]
                    pose[3, 3] = 1.0
                    
                    # 转换到激光雷达坐标系（完全按照BEVNet）
                    pose_lidar = np.matmul(Tr_inv, np.matmul(pose, Tr))
                    poses.append(pose_lidar)
        
        self.get_logger().info(f'Loaded {len(poses)} poses (converted to lidar frame)')
        return poses
    
    def load_kitti_bin(self, bin_path):
        """加载点云"""
        try:
            points = np.fromfile(bin_path, dtype=np.float32).reshape(-1, 4)
            return points
        except Exception as e:
            self.get_logger().error(f'Error loading {bin_path}: {e}')
            return None
    
    def create_pointcloud2_msg(self, points, frame_id='velodyne'):
        """创建PointCloud2消息"""
        msg = PointCloud2()
        
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = frame_id
        
        msg.height = 1
        msg.width = len(points)
        
        msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True
        msg.data = points.tobytes()
        
        return msg
    
    def matrix_to_pose(self, matrix):
        """变换矩阵转位姿 - 直接使用KITTI格式，不做变换"""
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = 'map'
        
        # KITTI格式的位姿矩阵是世界坐标系下的
        # 直接使用，不做任何变换
        pose.pose.position.x = matrix[0, 3]  # tx
        pose.pose.position.y = matrix[1, 3]  # ty
        pose.pose.position.z = matrix[2, 3]  # tz
        
        # 不做Z归一化，直接使用原始值
        # 如果需要归一化，应该在数据准备阶段完成
        
        # 从旋转矩阵提取四元数
        R = matrix[:3, :3]
        
        # 使用更稳定的四元数转换方法
        trace = np.trace(R)
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        # 归一化四元数
        norm = np.sqrt(w*w + x*x + y*y + z*z)
        if norm > 0:
            pose.pose.orientation.w = w / norm
            pose.pose.orientation.x = x / norm
            pose.pose.orientation.y = y / norm
            pose.pose.orientation.z = z / norm
        else:
            pose.pose.orientation.w = 1.0
            pose.pose.orientation.x = 0.0
            pose.pose.orientation.y = 0.0
            pose.pose.orientation.z = 0.0
        
        return pose
        
        # 旋转（简化的四元数转换）
        # 从旋转矩阵提取四元数
        R = matrix[:3, :3]
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s
        
        pose.pose.orientation.w = w
        pose.pose.orientation.x = x
        pose.pose.orientation.y = y
        pose.pose.orientation.z = z
        
        return pose
    
    def publish_tf(self, pose):
        """发布TF"""
        transforms = []
        
        # map -> velodyne (原有的)
        t1 = TransformStamped()
        t1.header.stamp = self.get_clock().now().to_msg()
        t1.header.frame_id = 'map'
        t1.child_frame_id = 'velodyne'
        t1.transform.translation.x = pose.pose.position.x
        t1.transform.translation.y = pose.pose.position.y
        t1.transform.translation.z = pose.pose.position.z
        t1.transform.rotation = pose.pose.orientation
        transforms.append(t1)
        
        # map -> odom (固定)
        t2 = TransformStamped()
        t2.header.stamp = self.get_clock().now().to_msg()
        t2.header.frame_id = 'map'
        t2.child_frame_id = 'odom'
        t2.transform.translation.x = 0.0
        t2.transform.translation.y = 0.0
        t2.transform.translation.z = 0.0
        t2.transform.rotation.w = 1.0
        transforms.append(t2)
        
        # odom -> base_link (跟随velodyne)
        t3 = TransformStamped()
        t3.header.stamp = self.get_clock().now().to_msg()
        t3.header.frame_id = 'odom'
        t3.child_frame_id = 'base_link'
        t3.transform.translation.x = pose.pose.position.x
        t3.transform.translation.y = pose.pose.position.y
        t3.transform.translation.z = 0.0
        t3.transform.rotation = pose.pose.orientation
        transforms.append(t3)
        
        # base_link -> velodyne (固定)
        t4 = TransformStamped()
        t4.header.stamp = self.get_clock().now().to_msg()
        t4.header.frame_id = 'base_link'
        t4.child_frame_id = 'velodyne'
        t4.transform.translation.x = 0.0
        t4.transform.translation.y = 0.0
        t4.transform.translation.z = 0.0
        t4.transform.rotation.w = 1.0
        transforms.append(t4)
        
        self.tf_broadcaster.sendTransform(transforms)
    
    def publish_frame(self):
        """发布当前帧"""
        if not self.playing:
            return
        
        if self.current_index >= len(self.bin_files):
            if self.loop:
                self.current_index = 0
                self.path_msg.poses.clear()
                self.get_logger().info('=== Looping ===')
            else:
                self.playing = False
                self.timer.cancel()
                self.get_logger().info('=== Complete ===')
            return
        
        # 1. 发布点云
        bin_file = self.bin_files[self.current_index]
        bin_path = os.path.join(self.velodyne_dir, bin_file)
        
        points = self.load_kitti_bin(bin_path)
        if points is not None:
            pc_msg = self.create_pointcloud2_msg(points)
            self.pc_pub.publish(pc_msg)
        
        # 2. 发布位姿
        if self.poses and self.current_index < len(self.poses):
            pose_matrix = self.poses[self.current_index]
            pose_msg = self.matrix_to_pose(pose_matrix)
            
            # 发布位姿
            self.pose_pub.publish(pose_msg)
            
            # 发布里程计
            odom_msg = Odometry()
            odom_msg.header = pose_msg.header
            odom_msg.child_frame_id = 'velodyne'
            odom_msg.pose.pose = pose_msg.pose
            self.odom_pub.publish(odom_msg)
            
            # 发布TF
            self.publish_tf(pose_msg)
            
            # 添加到路径
            self.path_msg.header.stamp = pose_msg.header.stamp
            self.path_msg.poses.append(pose_msg)
            self.path_pub.publish(self.path_msg)
        
        # 进度
        if self.current_index % 10 == 0:
            msg = f'Frame {self.current_index+1}/{len(self.bin_files)}'
            if points is not None:
                msg += f' ({len(points)} points)'
            self.get_logger().info(msg)
        
        self.current_index += 1

def main():
    parser = argparse.ArgumentParser(description='KITTI Player with Z Normalization')
    parser.add_argument('kitti_dir',
                       nargs='?',
                       default='/workspace/data/gyc/thesis/raw_demo_rosbag/bev_res_yc_fin_sl50tr1_final',
                       help='Path to KITTI directory')
    parser.add_argument('--sequence', '-s',
                       choices=['train', 'valid'],
                       default='valid',
                       help='Sequence to play')
    parser.add_argument('--rate', '-r',
                       type=float,
                       default=10.0,
                       help='Playback rate (Hz)')
    parser.add_argument('--loop', '-l',
                       action='store_true',
                       help='Loop playback')
    parser.add_argument('--no-normalize-z',
                       action='store_true',
                       help='Disable Z normalization')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        player = KittiPlayerNormalized(
            args.kitti_dir,
            args.sequence,
            args.rate,
            args.loop,
            normalize_z=not args.no_normalize_z
        )
        rclpy.spin(player)
    except KeyboardInterrupt:
        print('\n=== Stopping ===')
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
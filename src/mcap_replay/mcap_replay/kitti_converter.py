#!/usr/bin/env python3
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField, Image, CompressedImage
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf2_ros import TransformBroadcaster
import numpy as np
import os
import glob
from mcap import Writer
from mcap_ros2.writer import Writer as Ros2Writer
import struct
import cv2
from datetime import datetime, timezone

class KittiConverter(Node):
    def __init__(self):
        super().__init__('kitti_converter')
        
        # 参数
        self.declare_parameter('kitti_path', '')
        self.declare_parameter('output_path', 'output.mcap')
        self.declare_parameter('sequence', '00')
        self.declare_parameter('max_frames', -1)
        
        self.kitti_path = self.get_parameter('kitti_path').value
        self.output_path = self.get_parameter('output_path').value
        self.sequence = self.get_parameter('sequence').value
        self.max_frames = self.get_parameter('max_frames').value
        
        # KITTI相机内参（示例值，根据实际数据集调整）
        self.camera_matrix = np.array([
            [718.856, 0, 607.1928],
            [0, 718.856, 185.2157],
            [0, 0, 1]
        ])
        
        self.get_logger().info(f'Converting KITTI sequence {self.sequence} to MCAP')
        self.convert()
    
    def convert(self):
        """转换KITTI数据到MCAP"""
        # 路径设置
        sequence_path = os.path.join(self.kitti_path, 'sequences', self.sequence)
        velodyne_path = os.path.join(sequence_path, 'velodyne')
        image_path = os.path.join(sequence_path, 'image_2')
        pose_file = os.path.join(self.kitti_path, 'poses', f'{self.sequence}.txt')
        
        # 检查路径
        if not os.path.exists(velodyne_path):
            self.get_logger().error(f'Velodyne path not found: {velodyne_path}')
            return
        
        # 获取文件列表
        velodyne_files = sorted(glob.glob(os.path.join(velodyne_path, '*.bin')))
        image_files = sorted(glob.glob(os.path.join(image_path, '*.png')))
        
        # 读取位姿
        poses = self.load_poses(pose_file)
        
        # 确定要处理的帧数
        num_frames = len(velodyne_files)
        if self.max_frames > 0:
            num_frames = min(num_frames, self.max_frames)
        
        # 创建MCAP写入器
        with open(self.output_path, 'wb') as f:
            writer = Writer(f)
            ros2_writer = Ros2Writer(writer)
            
            # 写入元数据
            writer.start(profile='ros2', library='mcap-ros2-support')
            
            # 时间戳基准（使用当前时间）
            base_time = datetime.now(timezone.utc)
            
            for i in range(num_frames):
                # 计算时间戳（假设10Hz）
                timestamp_ns = int(base_time.timestamp() * 1e9) + i * int(1e8)
                
                # 处理点云
                if i < len(velodyne_files):
                    pointcloud_msg = self.load_pointcloud(velodyne_files[i], timestamp_ns)
                    ros2_writer.write_message(
                        topic='/velodyne_points',
                        message=pointcloud_msg,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns
                    )
                
                # 处理图像
                if i < len(image_files):
                    image_msg = self.load_image(image_files[i], timestamp_ns)
                    ros2_writer.write_message(
                        topic='/camera/image_raw',
                        message=image_msg,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns
                    )
                
                # 处理位姿
                if i < len(poses):
                    odom_msg = self.create_odometry(poses[i], timestamp_ns)
                    ros2_writer.write_message(
                        topic='/odom',
                        message=odom_msg,
                        log_time=timestamp_ns,
                        publish_time=timestamp_ns
                    )
                
                if i % 100 == 0:
                    self.get_logger().info(f'Processed {i}/{num_frames} frames')
            
            writer.finish()
        
        self.get_logger().info(f'Conversion complete. Output: {self.output_path}')
    
    def load_poses(self, pose_file):
        """加载KITTI位姿数据"""
        poses = []
        
        if not os.path.exists(pose_file):
            self.get_logger().warn(f'Pose file not found: {pose_file}')
            return poses
        
        with open(pose_file, 'r') as f:
            for line in f:
                values = [float(x) for x in line.strip().split()]
                # KITTI位姿是3x4矩阵，需要转换为4x4
                pose = np.eye(4)
                pose[:3, :] = np.array(values).reshape(3, 4)
                poses.append(pose)
        
        return poses
    
    def load_pointcloud(self, bin_file, timestamp_ns):
        """加载KITTI点云数据"""
        # 读取二进制文件
        points = np.fromfile(bin_file, dtype=np.float32).reshape(-1, 4)
        
        # 创建PointCloud2消息
        msg = PointCloud2()
        msg.header.stamp = self.timestamp_to_ros(timestamp_ns)
        msg.header.frame_id = 'velodyne'
        
        msg.height = 1
        msg.width = len(points)
        
        # 定义字段
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
        
        # 转换数据
        msg.data = points.tobytes()
        
        return msg
    
    def load_image(self, image_file, timestamp_ns):
        """加载图像数据"""
        # 读取图像
        img = cv2.imread(image_file)
        
        # 创建CompressedImage消息
        msg = CompressedImage()
        msg.header.stamp = self.timestamp_to_ros(timestamp_ns)
        msg.header.frame_id = 'camera'
        msg.format = 'jpeg'
        
        # 压缩图像
        _, compressed = cv2.imencode('.jpg', img, [cv2.IMWRITE_JPEG_QUALITY, 90])
        msg.data = compressed.tobytes()
        
        return msg
    
    def create_odometry(self, pose_matrix, timestamp_ns):
        """创建Odometry消息"""
        msg = Odometry()
        msg.header.stamp = self.timestamp_to_ros(timestamp_ns)
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        
        # 提取位置
        msg.pose.pose.position.x = pose_matrix[0, 3]
        msg.pose.pose.position.y = pose_matrix[1, 3]
        msg.pose.pose.position.z = pose_matrix[2, 3]
        
        # 提取旋转（从旋转矩阵转换为四元数）
        from scipy.spatial.transform import Rotation
        r = Rotation.from_matrix(pose_matrix[:3, :3])
        q = r.as_quat()  # [x, y, z, w]
        
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        
        # 设置协方差（示例值）
        msg.pose.covariance = [0.1] * 36
        msg.twist.covariance = [0.1] * 36
        
        return msg
    
    def timestamp_to_ros(self, timestamp_ns):
        """将纳秒时间戳转换为ROS时间"""
        from builtin_interfaces.msg import Time
        stamp = Time()
        stamp.sec = int(timestamp_ns // 1e9)
        stamp.nanosec = int(timestamp_ns % 1e9)
        return stamp

def main(args=None):
    rclpy.init(args=args)
    node = KittiConverter()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
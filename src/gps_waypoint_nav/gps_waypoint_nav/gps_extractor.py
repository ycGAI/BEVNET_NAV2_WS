#!/usr/bin/env python3
"""
Extract GPS Points from MCAP Files
从单个或多个MCAP文件中提取GPS轨迹点并保存
支持多文件拼接和去重
"""

import argparse
import os
import numpy as np
from pathlib import Path
import json
import csv
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py
from tqdm import tqdm
import matplotlib.pyplot as plt
from datetime import datetime
import glob

class GPSExtractor:
    def __init__(self, output_file="gps_path.json", downsample_distance=0.5):
        self.output_file = Path(output_file)
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        
        # 下采样距离（米）- 用于减少路径点密度
        self.downsample_distance = downsample_distance
        
        # 存储所有GPS点
        self.all_gps_points = []
        
        # 可能的GPS话题名称
        self.possible_gps_topics = [
            '/gps/fix',
            '/gps/data', 
            '/fix',
            '/gnss/fix',
            '/navsat/fix',
            '/gps/filtered',
            '/sensor/gps/fix',
            '/gps/position'
        ]

    def find_gps_topic(self, bag_path):
        """扫描bag文件，找到GPS话题"""
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", 
                output_serialization_format="cdr"
            ),
        )

        topic_types = reader.get_all_topics_and_types()
        topic_type_map = {topic.name: topic.type for topic in topic_types}
        
        print(f"\n📋 扫描文件: {bag_path}")
        print("  可用话题:")
        
        gps_topic = None
        for topic_name, msg_type in topic_type_map.items():
            print(f"    - {topic_name}: {msg_type}")
            
            # 检查是否是GPS相关话题
            if any(gps_key in topic_name.lower() for gps_key in ['gps', 'gnss', 'navsat', 'fix']):
                if 'NavSatFix' in msg_type or 'navsatfix' in msg_type.lower():
                    gps_topic = topic_name
                    print(f"  ✅ 找到GPS话题: {gps_topic}")
                    break
        
        del reader
        return gps_topic, topic_type_map.get(gps_topic) if gps_topic else None

    def extract_from_single_bag(self, bag_path):
        """从单个bag文件提取GPS数据"""
        # 首先找到GPS话题
        gps_topic, gps_msg_type = self.find_gps_topic(bag_path)
        
        if not gps_topic:
            print(f"⚠️  在 {bag_path} 中未找到GPS话题")
            return []
        
        # 读取GPS数据
        reader = rosbag2_py.SequentialReader()
        reader.open(
            rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
            rosbag2_py.ConverterOptions(
                input_serialization_format="cdr", 
                output_serialization_format="cdr"
            ),
        )
        
        gps_points = []
        print(f"\n📖 提取GPS数据从: {bag_path}")
        
        # 计数器
        total_messages = 0
        gps_messages = 0
        
        while reader.has_next():
            topic, data, timestamp = reader.read_next()
            total_messages += 1
            
            if topic == gps_topic:
                try:
                    msg_type = get_message(gps_msg_type)
                    msg = deserialize_message(data, msg_type)
                    
                    # 提取GPS坐标
                    if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                        gps_point = {
                            'timestamp': timestamp,
                            'timestamp_sec': timestamp / 1e9,  # 纳秒转秒
                            'latitude': msg.latitude,
                            'longitude': msg.longitude,
                            'altitude': msg.altitude if hasattr(msg, 'altitude') else 0.0,
                            'source_file': os.path.basename(bag_path)
                        }
                        
                        # 添加状态信息（如果有）
                        if hasattr(msg, 'status'):
                            gps_point['status'] = msg.status.status if hasattr(msg.status, 'status') else -1
                        
                        # 添加协方差信息（如果有）
                        if hasattr(msg, 'position_covariance'):
                            gps_point['position_covariance'] = list(msg.position_covariance)
                        
                        gps_points.append(gps_point)
                        gps_messages += 1
                        
                except Exception as e:
                    print(f"⚠️  解析GPS消息失败: {e}")
                    continue
        
        del reader
        
        print(f"  📊 处理消息: {total_messages} 总计, {gps_messages} GPS点")
        return gps_points

    def calculate_distance(self, p1, p2):
        """计算两个GPS点之间的距离（使用简单的欧几里得距离，单位：度）"""
        # 简单的经纬度距离计算（适用于小范围）
        # 1度纬度约111km，1度经度约111km * cos(纬度)
        lat_diff = (p2['latitude'] - p1['latitude']) * 111000  # 米
        lon_diff = (p2['longitude'] - p1['longitude']) * 111000 * np.cos(np.radians(p1['latitude']))
        return np.sqrt(lat_diff**2 + lon_diff**2)

    def downsample_points(self, points):
        """按距离下采样GPS点"""
        if not points or self.downsample_distance <= 0:
            return points
        
        downsampled = [points[0]]  # 始终保留第一个点
        last_kept = points[0]
        
        for point in points[1:]:
            dist = self.calculate_distance(last_kept, point)
            if dist >= self.downsample_distance:
                downsampled.append(point)
                last_kept = point
        
        # 始终保留最后一个点
        if len(points) > 1 and downsampled[-1] != points[-1]:
            downsampled.append(points[-1])
        
        print(f"  下采样: {len(points)} -> {len(downsampled)} 点 (距离阈值: {self.downsample_distance}m)")
        return downsampled

    def merge_and_sort_points(self, all_points):
        """合并和排序所有GPS点，去除重复"""
        if not all_points:
            return []
        
        # 按时间戳排序
        sorted_points = sorted(all_points, key=lambda x: x['timestamp'])
        
        # 去除完全重复的点（相同的经纬度）
        unique_points = []
        seen_coords = set()
        
        for point in sorted_points:
            coord_key = (round(point['latitude'], 8), round(point['longitude'], 8))
            if coord_key not in seen_coords:
                unique_points.append(point)
                seen_coords.add(coord_key)
        
        print(f"\n🔀 合并结果: {len(all_points)} -> {len(unique_points)} 个独特点")
        return unique_points

    def process_bags(self, bag_paths):
        """处理多个bag文件"""
        all_points = []
        
        for bag_path in bag_paths:
            if not os.path.exists(bag_path):
                print(f"⚠️  文件不存在: {bag_path}")
                continue
            
            points = self.extract_from_single_bag(bag_path)
            all_points.extend(points)
        
        # 合并、排序和去重
        merged_points = self.merge_and_sort_points(all_points)
        
        # 下采样
        self.all_gps_points = self.downsample_points(merged_points)
        
        return self.all_gps_points

    def save_as_json(self):
        """保存为JSON格式"""
        json_file = self.output_file.with_suffix('.json')
        
        data = {
            'metadata': {
                'total_points': len(self.all_gps_points),
                'downsample_distance': self.downsample_distance,
                'creation_time': datetime.now().isoformat(),
                'coordinate_system': 'WGS84'
            },
            'path': self.all_gps_points
        }
        
        with open(json_file, 'w') as f:
            json.dump(data, f, indent=2)
        
        print(f"✅ 保存JSON: {json_file}")

    def save_as_csv(self):
        """保存为CSV格式"""
        csv_file = self.output_file.with_suffix('.csv')
        
        if not self.all_gps_points:
            print("⚠️  没有GPS点可保存")
            return
        
        # 获取所有字段
        fieldnames = ['index', 'timestamp_sec', 'latitude', 'longitude', 'altitude', 'source_file']
        
        with open(csv_file, 'w', newline='') as f:
            writer = csv.DictWriter(f, fieldnames=fieldnames, extrasaction='ignore')
            writer.writeheader()
            
            for i, point in enumerate(self.all_gps_points):
                point_with_index = point.copy()
                point_with_index['index'] = i
                writer.writerow(point_with_index)
        
        print(f"✅ 保存CSV: {csv_file}")

    def save_as_yaml(self):
        """保存为ROS2兼容的YAML格式"""
        yaml_file = self.output_file.with_suffix('.yaml')
        
        # 简单的YAML格式（不使用外部库）
        with open(yaml_file, 'w') as f:
            f.write("# GPS Path for Global Planning\n")
            f.write(f"# Total points: {len(self.all_gps_points)}\n")
            f.write(f"# Downsample distance: {self.downsample_distance}m\n")
            f.write(f"# Creation time: {datetime.now().isoformat()}\n\n")
            
            f.write("gps_path:\n")
            f.write("  coordinate_system: \"WGS84\"\n")
            f.write(f"  total_points: {len(self.all_gps_points)}\n")
            f.write("  points:\n")
            
            for i, point in enumerate(self.all_gps_points):
                f.write(f"    - index: {i}\n")
                f.write(f"      latitude: {point['latitude']:.8f}\n")
                f.write(f"      longitude: {point['longitude']:.8f}\n")
                f.write(f"      altitude: {point['altitude']:.2f}\n")
                f.write(f"      timestamp_sec: {point['timestamp_sec']:.6f}\n")
        
        print(f"✅ 保存YAML: {yaml_file}")

    def plot_path(self):
        """可视化GPS路径"""
        if not self.all_gps_points:
            print("⚠️  没有GPS点可显示")
            return
        
        # 提取坐标
        lats = [p['latitude'] for p in self.all_gps_points]
        lons = [p['longitude'] for p in self.all_gps_points]
        
        plt.figure(figsize=(12, 8))
        
        # 绘制路径
        plt.subplot(1, 2, 1)
        plt.plot(lons, lats, 'b-', linewidth=1, alpha=0.6, label='GPS路径')
        plt.plot(lons, lats, 'r.', markersize=3, label='GPS点')
        plt.plot(lons[0], lats[0], 'go', markersize=10, label='起点')
        plt.plot(lons[-1], lats[-1], 'ro', markersize=10, label='终点')
        
        plt.xlabel('经度 (°)')
        plt.ylabel('纬度 (°)')
        plt.title(f'GPS路径 ({len(self.all_gps_points)} 点)')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.axis('equal')
        
        # 绘制高度变化
        if any(p.get('altitude', 0) != 0 for p in self.all_gps_points):
            plt.subplot(1, 2, 2)
            alts = [p.get('altitude', 0) for p in self.all_gps_points]
            distances = [0]
            for i in range(1, len(self.all_gps_points)):
                dist = self.calculate_distance(self.all_gps_points[i-1], self.all_gps_points[i])
                distances.append(distances[-1] + dist)
            
            plt.plot(distances, alts, 'b-', linewidth=2)
            plt.xlabel('累计距离 (m)')
            plt.ylabel('高度 (m)')
            plt.title('高度变化')
            plt.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # 保存图像
        plot_file = self.output_file.with_suffix('.png')
        plt.savefig(plot_file, dpi=150, bbox_inches='tight')
        print(f"✅ 保存图像: {plot_file}")
        
        plt.show()

    def print_statistics(self):
        """打印统计信息"""
        if not self.all_gps_points:
            print("⚠️  没有GPS数据")
            return
        
        print("\n📊 GPS路径统计:")
        print("=" * 50)
        print(f"  总点数: {len(self.all_gps_points)}")
        
        # 计算边界
        lats = [p['latitude'] for p in self.all_gps_points]
        lons = [p['longitude'] for p in self.all_gps_points]
        
        print(f"  纬度范围: {min(lats):.8f}° ~ {max(lats):.8f}°")
        print(f"  经度范围: {min(lons):.8f}° ~ {max(lons):.8f}°")
        
        # 计算总距离
        total_distance = 0
        for i in range(1, len(self.all_gps_points)):
            total_distance += self.calculate_distance(
                self.all_gps_points[i-1], 
                self.all_gps_points[i]
            )
        
        print(f"  总路径长度: {total_distance:.2f} m")
        
        # 时间范围
        if self.all_gps_points[0].get('timestamp_sec'):
            time_start = datetime.fromtimestamp(self.all_gps_points[0]['timestamp_sec'])
            time_end = datetime.fromtimestamp(self.all_gps_points[-1]['timestamp_sec'])
            duration = self.all_gps_points[-1]['timestamp_sec'] - self.all_gps_points[0]['timestamp_sec']
            
            print(f"  开始时间: {time_start}")
            print(f"  结束时间: {time_end}")
            print(f"  持续时间: {duration/60:.2f} 分钟")
        
        # 源文件统计
        source_files = set(p.get('source_file', 'unknown') for p in self.all_gps_points)
        print(f"  源文件数: {len(source_files)}")
        for sf in source_files:
            count = sum(1 for p in self.all_gps_points if p.get('source_file') == sf)
            print(f"    - {sf}: {count} 点")


def main():
    parser = argparse.ArgumentParser(
        description="从MCAP文件中提取GPS轨迹点"
    )
    parser.add_argument(
        "input", 
        nargs='+',
        help="输入MCAP文件路径（支持多个文件或通配符）"
    )
    parser.add_argument(
        "-o", "--output", 
        default="gps_path",
        help="输出文件路径（不含扩展名，默认: gps_path）"
    )
    parser.add_argument(
        "-d", "--downsample", 
        type=float, 
        default=0.5,
        help="下采样距离，单位：米（默认: 0.5m，设为0禁用）"
    )
    parser.add_argument(
        "--format", 
        choices=['all', 'json', 'csv', 'yaml'],
        default='all',
        help="输出格式（默认: all）"
    )
    parser.add_argument(
        "--plot", 
        action='store_true',
        help="显示路径可视化"
    )
    
    args = parser.parse_args()
    
    # 展开通配符
    input_files = []
    for pattern in args.input:
        matched_files = glob.glob(pattern)
        if matched_files:
            input_files.extend(matched_files)
        elif os.path.exists(pattern):
            input_files.append(pattern)
        else:
            print(f"⚠️  未找到匹配文件: {pattern}")
    
    if not input_files:
        print("❌ 没有找到有效的输入文件")
        return
    
    print(f"🚀 开始提取GPS路径")
    print(f"📁 输入文件: {len(input_files)} 个")
    for f in input_files:
        print(f"   - {f}")
    
    # 创建提取器
    extractor = GPSExtractor(
        output_file=args.output,
        downsample_distance=args.downsample
    )
    
    # 处理所有bag文件
    points = extractor.process_bags(input_files)
    
    if not points:
        print("❌ 没有提取到GPS点")
        return
    
    # 保存结果
    if args.format == 'all':
        extractor.save_as_json()
        extractor.save_as_csv()
        extractor.save_as_yaml()
    elif args.format == 'json':
        extractor.save_as_json()
    elif args.format == 'csv':
        extractor.save_as_csv()
    elif args.format == 'yaml':
        extractor.save_as_yaml()
    
    # 打印统计
    extractor.print_statistics()
    
    # 可视化
    if args.plot:
        extractor.plot_path()
    
    print("\n✅ GPS路径提取完成！")


if __name__ == "__main__":
    main()
# ros2 run gps_waypoint_nav extract_gps_from_mcap \
#   /path/to/bag.mcap \
#   -o ~/ros2_ws/src/gps_waypoint_nav/data/orchard_path
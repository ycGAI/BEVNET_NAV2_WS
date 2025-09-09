#!/usr/bin/env python3
"""
直接使用mcap库提取GPS数据
基于mcap_replay.py的方法
"""

import argparse
import json
import csv
import numpy as np
from pathlib import Path
from datetime import datetime
from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory

class GPSExtractor:
    def __init__(self, mcap_file, output_path, downsample_distance=0.5):
        self.mcap_file = mcap_file
        self.output_path = Path(output_path)
        self.downsample_distance = downsample_distance
        self.gps_points = []
        
    def read_mcap(self):
        """使用mcap库读取文件"""
        print(f"📖 读取MCAP文件: {self.mcap_file}")
        
        decoder_factory = DecoderFactory()
        
        try:
            with open(self.mcap_file, 'rb') as f:
                reader = make_reader(f, decoder_factories=[decoder_factory])
                
                # 获取话题信息
                channels = reader.get_summary().channels
                print("\n📋 可用话题:")
                
                gps_topic_id = None
                gps_topic_name = None
                
                for channel_id, channel in channels.items():
                    print(f"  - {channel.topic}: {channel.message_encoding}")
                    
                    # 查找GPS话题
                    if any(keyword in channel.topic.lower() for keyword in ['gps', 'fix', 'gnss', 'navsat']):
                        gps_topic_id = channel_id
                        gps_topic_name = channel.topic
                        print(f"  ✅ 找到GPS话题: {gps_topic_name}")
                        break
                
                if not gps_topic_name:
                    print("❌ 未找到GPS话题")
                    return False
                
                # 读取消息
                msg_count = 0
                print(f"\n📖 提取GPS数据...")
                
                for schema, channel, message in reader.iter_messages():
                    if channel.topic != gps_topic_name:
                        continue
                    
                    try:
                        # 解码消息
                        msg = decoder_factory.decoder_for(channel.message_encoding, schema)(message.data)
                        
                        # 提取GPS数据
                        if hasattr(msg, 'latitude') and hasattr(msg, 'longitude'):
                            gps_point = {
                                'timestamp': message.log_time,
                                'timestamp_sec': message.log_time / 1e9,
                                'latitude': msg.latitude,
                                'longitude': msg.longitude,
                                'altitude': msg.altitude if hasattr(msg, 'altitude') else 0.0
                            }
                            
                            # 添加状态信息
                            if hasattr(msg, 'status'):
                                gps_point['status'] = msg.status.status if hasattr(msg.status, 'status') else -1
                            
                            self.gps_points.append(gps_point)
                            msg_count += 1
                            
                            if msg_count % 100 == 0:
                                print(f"  已提取 {msg_count} 个GPS点...")
                                
                    except Exception as e:
                        continue
                
                print(f"✅ 提取完成: {msg_count} 个GPS点")
                return True
                
        except Exception as e:
            print(f"❌ 读取文件失败: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def calculate_distance(self, p1, p2):
        """计算两个GPS点之间的距离（米）"""
        lat_diff = (p2['latitude'] - p1['latitude']) * 111000
        lon_diff = (p2['longitude'] - p1['longitude']) * 111000 * np.cos(np.radians(p1['latitude']))
        return np.sqrt(lat_diff**2 + lon_diff**2)
    
    def downsample(self):
        """按距离下采样GPS点"""
        if not self.gps_points or self.downsample_distance <= 0:
            return
        
        original_count = len(self.gps_points)
        downsampled = [self.gps_points[0]]
        last_kept = self.gps_points[0]
        
        for point in self.gps_points[1:]:
            dist = self.calculate_distance(last_kept, point)
            if dist >= self.downsample_distance:
                downsampled.append(point)
                last_kept = point
        
        # 保留最后一个点
        if len(self.gps_points) > 1 and downsampled[-1] != self.gps_points[-1]:
            downsampled.append(self.gps_points[-1])
        
        self.gps_points = downsampled
        print(f"  下采样: {original_count} -> {len(self.gps_points)} 点 (距离阈值: {self.downsample_distance}m)")
    
    def save_data(self):
        """保存GPS数据"""
        if not self.gps_points:
            print("❌ 没有GPS数据可保存")
            return
        
        # 创建输出目录
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        
        # JSON格式
        json_file = self.output_path.with_suffix('.json')
        data = {
            'metadata': {
                'total_points': len(self.gps_points),
                'downsample_distance': self.downsample_distance,
                'creation_time': datetime.now().isoformat(),
                'coordinate_system': 'WGS84'
            },
            'path': self.gps_points
        }
        
        with open(json_file, 'w') as f:
            json.dump(data, f, indent=2)
        print(f"✅ 保存JSON: {json_file}")
        
        # CSV格式
        csv_file = self.output_path.with_suffix('.csv')
        with open(csv_file, 'w', newline='') as f:
            fieldnames = ['index', 'timestamp_sec', 'latitude', 'longitude', 'altitude']
            writer = csv.DictWriter(f, fieldnames=fieldnames)
            writer.writeheader()
            
            for i, point in enumerate(self.gps_points):
                writer.writerow({
                    'index': i,
                    'timestamp_sec': point['timestamp_sec'],
                    'latitude': point['latitude'],
                    'longitude': point['longitude'],
                    'altitude': point.get('altitude', 0.0)
                })
        print(f"✅ 保存CSV: {csv_file}")
        
        # YAML格式
        yaml_file = self.output_path.with_suffix('.yaml')
        with open(yaml_file, 'w') as f:
            f.write("# GPS Path for Navigation\n")
            f.write(f"# Total points: {len(self.gps_points)}\n")
            f.write(f"# Creation time: {datetime.now().isoformat()}\n\n")
            f.write("gps_path:\n")
            f.write("  coordinate_system: \"WGS84\"\n")
            f.write(f"  total_points: {len(self.gps_points)}\n")
            f.write("  points:\n")
            
            for i, point in enumerate(self.gps_points):
                f.write(f"    - index: {i}\n")
                f.write(f"      latitude: {point['latitude']:.8f}\n")
                f.write(f"      longitude: {point['longitude']:.8f}\n")
                f.write(f"      altitude: {point.get('altitude', 0):.2f}\n")
        print(f"✅ 保存YAML: {yaml_file}")
    
    def print_statistics(self):
        """打印统计信息"""
        if not self.gps_points:
            return
        
        print("\n📊 GPS路径统计:")
        print("=" * 50)
        print(f"  总点数: {len(self.gps_points)}")
        
        lats = [p['latitude'] for p in self.gps_points]
        lons = [p['longitude'] for p in self.gps_points]
        
        print(f"  纬度范围: {min(lats):.8f}° ~ {max(lats):.8f}°")
        print(f"  经度范围: {min(lons):.8f}° ~ {max(lons):.8f}°")
        
        # 计算总距离
        total_distance = 0
        for i in range(1, len(self.gps_points)):
            total_distance += self.calculate_distance(
                self.gps_points[i-1], 
                self.gps_points[i]
            )
        print(f"  总路径长度: {total_distance:.2f} m")
        
        # 时间范围
        if self.gps_points[0].get('timestamp_sec'):
            duration = self.gps_points[-1]['timestamp_sec'] - self.gps_points[0]['timestamp_sec']
            print(f"  持续时间: {duration/60:.2f} 分钟")
    
    def extract(self):
        """执行提取流程"""
        print(f"🚀 开始提取GPS数据")
        print(f"📁 输入文件: {self.mcap_file}")
        
        # 读取MCAP
        if not self.read_mcap():
            return False
        
        # 下采样
        if self.downsample_distance > 0:
            self.downsample()
        
        # 保存数据
        self.save_data()
        
        # 打印统计
        self.print_statistics()
        
        print("\n✅ GPS提取完成！")
        return True

def main():
    parser = argparse.ArgumentParser(description="从MCAP文件提取GPS轨迹")
    parser.add_argument("input", help="输入MCAP文件路径")
    parser.add_argument("-o", "--output", default="gps_path", help="输出文件路径（不含扩展名）")
    parser.add_argument("-d", "--downsample", type=float, default=0.5, 
                       help="下采样距离（米），0表示不下采样")
    
    args = parser.parse_args()
    
    extractor = GPSExtractor(
        mcap_file=args.input,
        output_path=args.output,
        downsample_distance=args.downsample
    )
    
    extractor.extract()

if __name__ == "__main__":
    main()
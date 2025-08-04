#!/usr/bin/env python3
"""
MCAP 文件话题查看器 - 使用纯 mcap 库
不依赖 ROS2 环境
"""

import sys
from pathlib import Path
from collections import defaultdict

# 首先安装 mcap: pip3 install mcap mcap-ros2-support

try:
    from mcap.reader import make_reader
    from mcap_ros2.reader import read_ros2_messages
except ImportError:
    print("❌ 需要安装 mcap 库:")
    print("   pip3 install mcap mcap-ros2-support")
    sys.exit(1)

def format_bytes(size):
    """格式化字节大小"""
    for unit in ['B', 'KB', 'MB', 'GB']:
        if size < 1024.0:
            return f"{size:.2f} {unit}"
        size /= 1024.0
    return f"{size:.2f} TB"

def check_mcap_file(mcap_file):
    """检查 MCAP 文件内容"""
    if not Path(mcap_file).exists():
        print(f"❌ 文件不存在: {mcap_file}")
        return
    
    file_size = Path(mcap_file).stat().st_size
    print(f"📁 MCAP 文件: {mcap_file}")
    print(f"📊 文件大小: {format_bytes(file_size)}")
    print("=" * 80)
    
    # 话题统计
    topic_stats = defaultdict(lambda: {
        'count': 0,
        'msg_type': '',
        'first_time': float('inf'),
        'last_time': 0,
        'total_size': 0
    })
    
    try:
        with open(mcap_file, 'rb') as f:
            reader = make_reader(f)
            
            # 获取摘要信息
            summary = reader.get_summary()
            if summary:
                print(f"\n📈 文件摘要:")
                print(f"   消息总数: {summary.statistics.message_count:,}")
                print(f"   通道数: {summary.statistics.channel_count}")
                print(f"   架构数: {summary.statistics.schema_count}")
                
                # 时间信息
                if summary.statistics.message_start_time and summary.statistics.message_end_time:
                    start_sec = summary.statistics.message_start_time / 1e9
                    end_sec = summary.statistics.message_end_time / 1e9
                    duration = end_sec - start_sec
                    print(f"   录制时长: {duration:.2f} 秒")
            
            print("\n🔍 分析话题详情...")
            
            # 读取并统计消息
            message_count = 0
            
            # 尝试使用 ros2 消息读取器
            try:
                f.seek(0)  # 重置文件指针
                reader = make_reader(f)
                
                for schema, channel, message, ros_msg in read_ros2_messages(reader):
                    topic = channel.topic
                    msg_type = schema.name
                    
                    stats = topic_stats[topic]
                    stats['count'] += 1
                    stats['msg_type'] = msg_type
                    stats['first_time'] = min(stats['first_time'], message.log_time)
                    stats['last_time'] = max(stats['last_time'], message.log_time)
                    stats['total_size'] += len(message.data)
                    
                    message_count += 1
                    if message_count % 5000 == 0:
                        print(f"   已处理 {message_count:,} 条消息...", end='\r')
                
                print(f"   已处理 {message_count:,} 条消息...完成!    ")
                
            except Exception as e:
                print(f"⚠️  使用 ROS2 reader 失败，尝试基础方法: {e}")
                
                # 回退到基础方法
                f.seek(0)
                reader = make_reader(f, decoder_factories=[])
                
                # 收集 schema 和 channel 信息
                schemas = {}
                channels = {}
                
                for schema, channel, message in reader.iter_messages():
                    if schema.id not in schemas:
                        schemas[schema.id] = schema
                    if channel.id not in channels:
                        channels[channel.id] = channel
                        
                    topic = channel.topic
                    
                    stats = topic_stats[topic]
                    stats['count'] += 1
                    stats['msg_type'] = schemas.get(channel.schema_id, schema).name if channel.schema_id else 'Unknown'
                    stats['first_time'] = min(stats['first_time'], message.log_time)
                    stats['last_time'] = max(stats['last_time'], message.log_time)
                    stats['total_size'] += len(message.data)
                    
                    message_count += 1
                    if message_count % 5000 == 0:
                        print(f"   已处理 {message_count:,} 条消息...", end='\r')
                
                print(f"   已处理 {message_count:,} 条消息...完成!    ")
        
        # 显示结果
        print(f"\n📋 话题列表 (共 {len(topic_stats)} 个话题):")
        print("-" * 100)
        print(f"{'话题名称':<50} {'消息类型':<30} {'消息数':<10} {'频率(Hz)':<10} {'大小':<10}")
        print("-" * 100)
        
        # 按话题名排序
        for topic in sorted(topic_stats.keys()):
            stats = topic_stats[topic]
            
            # 计算频率
            if stats['count'] > 1 and stats['first_time'] != float('inf'):
                duration = (stats['last_time'] - stats['first_time']) / 1e9
                freq = stats['count'] / duration if duration > 0 else 0
            else:
                freq = 0
            
            # 格式化输出
            topic_short = topic if len(topic) <= 48 else topic[:45] + "..."
            type_short = stats['msg_type'] if len(stats['msg_type']) <= 28 else stats['msg_type'][:25] + "..."
            
            print(f"{topic_short:<50} {type_short:<30} {stats['count']:<10} {freq:<10.1f} {format_bytes(stats['total_size']):<10}")
        
        # 分类显示
        print("\n📊 按类型分类:")
        print("-" * 50)
        
        # 图像话题
        image_topics = [(t, s) for t, s in topic_stats.items() 
                       if 'image' in t.lower() or 'Image' in s['msg_type']]
        if image_topics:
            print(f"\n📷 图像话题 ({len(image_topics)} 个):")
            for topic, stats in sorted(image_topics):
                print(f"   {topic} - {stats['count']} 条消息 ({stats['msg_type']})")
        
        # 点云话题
        pc_topics = [(t, s) for t, s in topic_stats.items() 
                    if 'point' in t.lower() or 'PointCloud' in s['msg_type'] or 'velodyne' in t.lower()]
        if pc_topics:
            print(f"\n☁️  点云话题 ({len(pc_topics)} 个):")
            for topic, stats in sorted(pc_topics):
                print(f"   {topic} - {stats['count']} 条消息 ({stats['msg_type']})")
        
        # 里程计话题
        odom_topics = [(t, s) for t, s in topic_stats.items() 
                      if 'odom' in t.lower() or 'Odometry' in s['msg_type']]
        if odom_topics:
            print(f"\n📍 里程计话题 ({len(odom_topics)} 个):")
            for topic, stats in sorted(odom_topics):
                print(f"   {topic} - {stats['count']} 条消息 ({stats['msg_type']})")
        
        # TF话题
        tf_topics = [(t, s) for t, s in topic_stats.items() 
                    if t in ['/tf', '/tf_static'] or 'tf2_msgs' in s['msg_type']]
        if tf_topics:
            print(f"\n🔄 TF话题 ({len(tf_topics)} 个):")
            for topic, stats in sorted(tf_topics):
                print(f"   {topic} - {stats['count']} 条消息 ({stats['msg_type']})")
        
    except Exception as e:
        print(f"❌ 读取 MCAP 文件失败: {e}")
        import traceback
        traceback.print_exc()

def main():
    if len(sys.argv) < 2:
        print("用法: python3 mcap_viewer.py <mcap_file>")
        print("示例: python3 mcap_viewer.py data/mcap/rosbag2_2025_07_30-11_09_54_0.mcap")
        sys.exit(1)
    
    mcap_file = sys.argv[1]
    check_mcap_file(mcap_file)

if __name__ == "__main__":
    main()
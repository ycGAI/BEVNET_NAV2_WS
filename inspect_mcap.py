#!/usr/bin/env python3
import sys
from mcap.reader import make_reader
from mcap_ros2.reader import read_ros2_messages

mcap_file = "data/mcap/rosbag2_2025_07_30-11_09_54_0.mcap"

print(f"Inspecting MCAP file: {mcap_file}")
print("=" * 50)

topics = {}
message_count = 0

try:
    with open(mcap_file, 'rb') as f:
        reader = make_reader(f)
        
        # 统计话题
        for schema, channel, message, ros_msg in read_ros2_messages(reader):
            if channel.topic not in topics:
                topics[channel.topic] = {
                    'type': str(type(ros_msg).__name__),
                    'count': 0
                }
            topics[channel.topic]['count'] += 1
            message_count += 1
            
            # 只读取前1000条消息
            if message_count >= 1000:
                print(f"\n(Stopped after {message_count} messages)")
                break

    print(f"\nTotal messages scanned: {message_count}")
    print(f"\nTopics found:")
    for topic, info in sorted(topics.items()):
        print(f"  {topic}: {info['type']} ({info['count']} messages)")

    # 查找点云话题
    pointcloud_topics = [t for t in topics if 'PointCloud2' in topics[t]['type']]
    if pointcloud_topics:
        print(f"\nPointCloud2 topics found: {pointcloud_topics}")
    else:
        print("\nNo PointCloud2 topics found in first 1000 messages")
        
except Exception as e:
    print(f"Error: {e}")
    import traceback
    traceback.print_exc()

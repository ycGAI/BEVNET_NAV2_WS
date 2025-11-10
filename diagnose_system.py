#!/usr/bin/env python3
"""
BEVNet Navigation System Diagnostics
诊断系统问题并提供解决方案
"""

import subprocess
import time


def run_command(cmd):
    """运行命令并返回输出"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=5)
        return result.stdout, result.stderr, result.returncode
    except subprocess.TimeoutExpired:
        return "", "Timeout", 1


def check_ros2_topics():
    """检查必要的ROS2话题"""
    print("\n=== 检查ROS2话题 ===")
    
    required_topics = [
        "/velodyne_points",
        "/bevnet/costmap",
        "/odom",
        "/tf",
        "/global_costmap/costmap",
        "/local_costmap/costmap",
        "/plan",
        "/local_plan",
        "/gps_path",
        "/current_goal"
    ]
    
    stdout, _, _ = run_command("ros2 topic list")
    available_topics = stdout.split('\n')
    
    for topic in required_topics:
        if topic in available_topics:
            print(f"✓ {topic}")
        else:
            print(f"✗ {topic} - 缺失!")
    
    return available_topics


def check_tf_tree():
    """检查TF树"""
    print("\n=== 检查TF树 ===")
    
    required_transforms = [
        ("map", "odom"),
        ("odom", "base_link"),
        ("base_link", "velodyne")
    ]
    
    print("运行: ros2 run tf2_tools view_frames.py")
    stdout, stderr, _ = run_command("timeout 2 ros2 run tf2_tools view_frames.py")
    
    if "Listening to tf data" in stderr:
        print("✓ TF系统运行中")
    else:
        print("✗ TF系统可能有问题")
    
    # 检查具体的transform
    for parent, child in required_transforms:
        stdout, _, returncode = run_command(f"ros2 run tf2_ros tf2_echo {parent} {child}")
        if returncode == 0:
            print(f"✓ {parent} -> {child}")
        else:
            print(f"✗ {parent} -> {child} - 缺失!")


def check_nav2_status():
    """检查Nav2状态"""
    print("\n=== 检查Nav2状态 ===")
    
    services = [
        "/controller_server/get_state",
        "/planner_server/get_state",
        "/bt_navigator/get_state"
    ]
    
    for service in services:
        stdout, _, returncode = run_command(f"ros2 service list | grep {service}")
        if service in stdout:
            print(f"✓ {service.split('/')[1]} 运行中")
        else:
            print(f"✗ {service.split('/')[1]} 未运行!")


def check_costmap_publishing():
    """检查代价地图发布"""
    print("\n=== 检查代价地图发布 ===")
    
    costmaps = [
        "/bevnet/costmap",
        "/global_costmap/costmap",
        "/local_costmap/costmap"
    ]
    
    for costmap in costmaps:
        # 先检查话题是否存在
        stdout, _, _ = run_command("ros2 topic list")
        if costmap in stdout:
            # 用echo检查是否有数据，timeout更短
            stdout, _, returncode = run_command(f"timeout 1 ros2 topic echo {costmap} --once")
            if returncode == 0 and len(stdout) > 0:
                print(f"✓ {costmap}: 有数据发布")
            else:
                print(f"⚠ {costmap}: 话题存在但可能无数据")
        else:
            print(f"✗ {costmap}: 话题不存在")


def provide_solutions():
    """提供解决方案"""
    print("\n=== 建议的解决方案 ===")
    
    print("""
    1. 如果TF树缺失:
       - 运行: python3 /workspace/bevnet_nav2_ws/src/tf_publisher.py
       
    2. 如果BEVNet costmap不发布:
       - 检查点云话题是否正常
       - 确认模型文件路径正确
       - 检查frame_id设置
       
    3. 如果Nav2规划失败:
       - 检查global_costmap是否有数据
       - 确认allow_unknown参数为true
       - 降低机器人速度参数
       
    4. 如果看不到local_plan:
       - 在RViz中订阅 /local_plan (不是/local_path)
       - 检查controller_server是否运行
       
    5. 完整启动顺序:
       a) 启动数据发布 (kitti_replayer)
       b) 启动TF发布器
       c) 启动BEVNet推理
       d) 启动Nav2
       e) 启动GPS跟随器
       
    6. 使用集成启动脚本:
       bash /workspace/bevnet_nav2_ws/launch_system.sh
    """)


def main():
    print("=" * 50)
    print("BEVNet Navigation System Diagnostics")
    print("=" * 50)
    
    # 运行诊断
    check_ros2_topics()
    check_tf_tree()
    check_nav2_status()
    check_costmap_publishing()
    
    # 提供解决方案
    provide_solutions()
    
    print("\n诊断完成!")


if __name__ == "__main__":
    main()
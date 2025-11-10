#!/bin/bash
# BEVNet Navigation System Launch Script

echo "========================================="
echo "Starting BEVNet Navigation System"
echo "========================================="

# 设置环境变量
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH

# 创建多个终端或使用screen/tmux
# 这里使用后台进程方式

# 终端1: 发布KITTI格式的数据
echo "Starting KITTI data publisher..."
python3 ./src/kitti_replayer.py \
    /workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/ \
    --sequence train \
    --loop &
KITTI_PID=$!
sleep 2

# 终端2: TF发布器（改进版）
echo "Starting Improved TF publisher..."
python3 /workspace/bevnet_nav2_ws/src/tf_publisher.py \
    --ros-args \
    -p poses_file:=/workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/sequences/train/poses.txt &
TF_PID=$!
sleep 1

# 终端3: BEVNet推理节点
echo "Starting BEVNet inference node..."
python3 /workspace/bevnet_nav2_ws/src/bevnet_nav2_core/bevnet_nav2_core/bevnet_inference_node.py \
    /workspace/bevnet_nav2_ws/models/best.pth.34 \
    --ros-args \
    -p publish_costmap:=true \
    -p visualize:=true &
BEVNET_PID=$!
sleep 3

# 终端4: Nav2导航
echo "Starting Nav2..."
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=False \
    params_file:=/workspace/bevnet_nav2_ws/config/nav2_params.yaml &
NAV2_PID=$!
sleep 5

# 终端5: GPS路径跟随
echo "Starting GPS waypoint follower..."
python3 /workspace/bevnet_nav2_ws/src/gps_waypoint_nav/gps_waypoint_nav/gps_waypoint_follower.py \
    --ros-args \
    -p poses_file:=/workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/sequences/train/poses.txt \
    -p waypoint_spacing:=5.0 \
    -p goal_tolerance:=2.0 \
    -p loop:=true &
GPS_PID=$!

echo "========================================="
echo "All nodes started!"
echo "Process IDs:"
echo "  KITTI Publisher: $KITTI_PID"
echo "  TF Publisher: $TF_PID"
echo "  BEVNet: $BEVNET_PID"
echo "  Nav2: $NAV2_PID"
echo "  GPS Follower: $GPS_PID"
echo "========================================="
echo "Press Ctrl+C to stop all nodes"

# 等待用户中断
trap "echo 'Stopping all nodes...'; kill $KITTI_PID $TF_PID $BEVNET_PID $NAV2_PID $GPS_PID 2>/dev/null; exit" INT

# 保持脚本运行
while true; do
    sleep 1
done
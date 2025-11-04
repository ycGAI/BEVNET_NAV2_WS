cd ~/bevnet_nav2_ws
./docker_run.sh

#独立模式运行bevnet
# 设置Python路径
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH

# 进入工作目录
cd /workspace/bevnet_nav2_ws

# 运行可视化测试
python3 run_bevnet_standalone.py --fps 5


# ros2 
# 设置环境
source /opt/ros/foxy/setup.bash
source install/setup.bash
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH

# 终端1：运行点云发布器（测试用）
python3 test_bevnet_ros2.py

# 终端2：运行BEVNet推理节点
ros2 run bevnet_nav2_core bevnet_inference_node.py

# 终端3：查看结果
ros2 topic list
ros2 topic hz /bevnet/costmap
ros2 topic echo /bevnet/costmap -n 1

# 使用mcap数据
# 查找MCAP文件
find /workspace/data -name "*.mcap" -type f

# 处理MCAP文件
python3 process_mcap.py --mcap /path/to/file.mcap --max-frames 10





====================== 最新版本 ===============================
# 终端1 发布kitti格式的数据
python ./src/kitti_replayer.py /workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/ --sequence train --loop

# 终端2 bevnet推理节点
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH
python3 /workspace/bevnet_nav2_ws/src/bevnet_nav2_core/bevnet_nav2_core/bevnet_inference_node.py /workspace/bevnet_nav2_ws/models/best.pth.34

# 终端3 nav2 导航
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=False \
    params_file:=/workspace/bevnet_nav2_ws/config/nav2_params.yaml

# 终端4 gps路径跟随
python3 src/gps_waypoint_nav/gps_waypoint_nav/gps_waypoint_follower.py \
    --ros-args \
    -p poses_file:=/workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/sequences/train/poses.txt \
    -p waypoint_spacing:=5.0 \
    -p goal_tolerance:=2.0 \
    -p loop:=true
或者source install/setup.bash  # 如果已经编译

# 方法2：使用ros2 run（需要先colcon build）
ros2 run gps_waypoint_nav gps_waypoint_follower \
    --ros-args \
    -p poses_file:=/workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/sequences/train/poses.txt \
    -p waypoint_spacing:=5.0 \
    -p goal_tolerance:=2.0 \
    -p loop:=true
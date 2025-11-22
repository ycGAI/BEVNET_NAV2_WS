cd ~/bevnet_nav2_ws
./docker_run.sh

#独立模式运行bevnet
# 设置Python路径
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH

# 进入工作目录
cd /workspace/bevnet_nav2_ws

# 运行可视化测试
python3 run_bevnet_standalone.py --fps 5



# 终端1：运行点云发布器（测试用）
python3 test_bevnet_ros2.py

# 终端2：运行BEVNet推理节点
ros2 run bevnet_nav2_core bevnet_inference_node.py

# 终端3：查看结果
ros2 topic list
ros2 topic hz /bevnet/costmap
ros2 topic echo /bevnet/costmap -n 1




====================== 最新版本 ===============================
# 终端1
python3 /workspace/bevnet_nav2_ws/src/tf_publisher.py

# 终端2 发布kitti格式的数据
python ./src/kitti_replayer.py /workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/ --sequence train --loop

# 终端3 bevnet推理节点
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH
python3 /workspace/bevnet_nav2_ws/src/bevnet_nav2_core/bevnet_nav2_core/bevnet_inference_node.py /workspace/bevnet_nav2_ws/models/best.pth.41


# 终端4 nav2 导航
ros2 launch nav2_bringup navigation_launch.py \
    use_sim_time:=False \
    params_file:=/workspace/bevnet_nav2_ws/config/nav2_params_bevnet_only.yaml

# 终端5 gps路径跟随
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



# 重启后，可以这样直接启一个容器
# 1. 必须先运行这个（重启后会失效）
xhost +local:

# 2. 然后运行完整命令
XAUTHORITY=$HOME/.Xauthority
docker run -it --gpus all \
  --env="DISPLAY=$DISPLAY" \
  --env="QT_X11_NO_MITSHM=1" \
  --env="XAUTHORITY=/root/.Xauthority" \
  --volume="$XAUTHORITY:/root/.Xauthority:rw" \
  --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
  --volume="/home/gyc/bevnet_nav2_ws:/workspace/bevnet_nav2_ws" \
  --volume="/home/gyc/semantic_bevnet:/workspace/bevnet" \
  --volume="/media/gyc/Backup Plus8:/workspace/data" \
  --network=host \
  --privileged \
  bevnet_modi:ws1 \
  bash
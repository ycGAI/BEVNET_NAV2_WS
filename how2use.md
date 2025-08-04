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
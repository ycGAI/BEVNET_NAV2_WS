1.mcap
2.bevnet inference node
3.bevmap to nav2
4.path vis


mcap player发布消息：

# 安装 rosbag2 MCAP 支持
sudo apt update
sudo apt install -y ros-humble-rosbag2-storage-mcap

# 安装 Python MCAP 库
pip3 install mcap mcap-ros2-support



source /opt/ros/foxy/setup.bash
cd /workspace/bevnet_nav2_ws
source install/setup.bash

# 使用实际的 MCAP 文件路径
LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 ros2 run mcap_replay mcap_player --ros-args \
  -p mcap_file:=/workspace/data/raw_demo_rosbag/row_coverage_1_0.mcap \
  -p loop:=true \
  -p rate:=0.1


第二个终端：
bevnet推理节点：
source /opt/ros/foxy/setup.bash
cd /workspace/bevnet_nav2_ws
export PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH

# 使用正确的模型路径
python3 src/bevnet_nav2_core/bevnet_nav2_core/bevnet_inference_node.py --ros-args \
  -p model_path:=/workspace/models/best.pth.34
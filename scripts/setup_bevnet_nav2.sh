#!/bin/bash

# 设置BEVNet和Nav2环境的脚本

echo "Setting up BEVNet Nav2 environment..."

# 设置环境变量
export PYTHONPATH=/workspace/bevnet:$PYTHONPATH
export ROS_DISTRO=${ROS_DISTRO:-humble}

# Source ROS环境
source /opt/ros/${ROS_DISTRO}/setup.bash

# 如果工作空间已经构建，source它
if [ -f /opt/ros_ws/install/setup.bash ]; then
    source /opt/ros_ws/install/setup.bash
fi

# 构建工作空间的函数
build_workspace() {
    echo "Building ROS workspace..."
    cd /opt/ros_ws
    
    # 复制源代码
    if [ -d /workspace/bevnet_nav2_ws/src ]; then
        cp -r /workspace/bevnet_nav2_ws/src/* /opt/ros_ws/src/
    fi
    
    # 安装依赖
    rosdep update
    rosdep install --from-paths src --ignore-src -r -y
    
    # 构建
    colcon build --symlink-install
    
    # Source安装文件
    source install/setup.bash
    
    echo "Workspace built successfully!"
}

# 运行BEVNet推理节点的函数
run_bevnet_inference() {
    local model_path=${1:-"/workspace/models/kitti4_single_model.pth"}
    local model_type=${2:-"single"}
    
    echo "Starting BEVNet inference node..."
    echo "Model path: $model_path"
    echo "Model type: $model_type"
    
    ros2 run bevnet_nav2 bevnet_inference_node.py \
        --ros-args \
        -p model_path:="$model_path" \
        -p model_type:="$model_type"
}

# 使用launch文件启动的函数
launch_bevnet() {
    local model_path=${1:-"/workspace/models/kitti4_single_model.pth"}
    local model_type=${2:-"single"}
    
    echo "Launching BEVNet with Nav2..."
    
    ros2 launch bevnet_nav2 bevnet_inference.launch.py \
        model_path:="$model_path" \
        model_type:="$model_type"
}

# 显示帮助信息
show_help() {
    echo "BEVNet Nav2 Setup Script"
    echo ""
    echo "Commands:"
    echo "  source setup_bevnet_nav2.sh          - Setup environment"
    echo "  build_workspace                      - Build ROS workspace"
    echo "  run_bevnet_inference [model] [type]  - Run inference node"
    echo "  launch_bevnet [model] [type]         - Launch with Nav2"
    echo ""
    echo "Model types: single, recurrent"
}

echo "Environment setup complete!"
echo "Available commands: build_workspace, run_bevnet_inference, launch_bevnet"
echo "Run 'show_help' for more information"
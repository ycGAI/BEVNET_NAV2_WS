#!/bin/bash

# Docker运行脚本，用于启动BEVNet Nav2容器

# 默认参数
CONTAINER_NAME="bevnet_nav2_container"
IMAGE_NAME="bevnet_modi:v1"
MODEL_PATH=""
MODEL_TYPE="single"

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --model)
            MODEL_PATH="$2"
            shift 2
            ;;
        --type)
            MODEL_TYPE="$2"
            shift 2
            ;;
        --container-name)
            CONTAINER_NAME="$2"
            shift 2
            ;;
        --image)
            IMAGE_NAME="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "Options:"
            echo "  --model PATH         Path to model weights file"
            echo "  --type TYPE          Model type: single or recurrent (default: single)"
            echo "  --container-name NAME Container name (default: bevnet_nav2_container)"
            echo "  --image NAME         Docker image name (default: bevnet_modi:v1)"
            echo "  --help               Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# 检查是否已有容器在运行
if docker ps -a | grep -q "$CONTAINER_NAME"; then
    echo "Stopping existing container: $CONTAINER_NAME"
    docker stop "$CONTAINER_NAME"
    docker rm "$CONTAINER_NAME"
fi

# 启动容器
echo "Starting BEVNet Nav2 container..."
echo "Image: $IMAGE_NAME"
echo "Container: $CONTAINER_NAME"

docker run -it --rm \
    --name "$CONTAINER_NAME" \
    --gpus all \
    --network host \
    --ipc host \
    --pid host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v /home/gyc/semantic_bevnet:/workspace/bevnet:rw \
    -v "/media/gyc/Backup Plus7/gyc/thesis":/workspace/data:rw \
    -v /home/gyc/bevnet_nav2_ws:/workspace/bevnet_nav2_ws:rw \
    -v /home/gyc/bevnet_nav2_ws/src:/opt/ros_ws/src:rw \
    -v /home/gyc/bevnet_nav2_ws/models:/workspace/models:rw \
    -v /home/gyc/bevnet_nav2_ws/scripts:/workspace/scripts:rw \
    --workdir /opt/ros_ws \
    "$IMAGE_NAME" \
    bash -c "
        # 设置环境
        source /opt/ros/\${ROS_DISTRO}/setup.bash
        export PYTHONPATH=/workspace/bevnet:\$PYTHONPATH
        
        # 复制推理脚本到正确位置
        mkdir -p /opt/ros_ws/src/bevnet_nav2/scripts
        cp /workspace/bevnet_nav2_ws/src/bevnet_nav2/scripts/bevnet_inference_node.py /opt/ros_ws/src/bevnet_nav2/scripts/
        chmod +x /opt/ros_ws/src/bevnet_nav2/scripts/bevnet_inference_node.py
        
        # 构建工作空间（如果需要）
        if [ ! -f /opt/ros_ws/install/setup.bash ]; then
            echo 'Building workspace...'
            cd /opt/ros_ws
            colcon build --symlink-install
        fi
        
        # Source工作空间
        source /opt/ros_ws/install/setup.bash
        
        # 如果提供了模型路径，直接运行推理
        if [ -n '$MODEL_PATH' ]; then
            echo 'Starting BEVNet inference...'
            echo 'Model: $MODEL_PATH'
            echo 'Type: $MODEL_TYPE'
            ros2 run bevnet_nav2 bevnet_inference_node.py \\
                --ros-args \\
                -p model_path:='$MODEL_PATH' \\
                -p model_type:='$MODEL_TYPE'
        else
            echo 'BEVNet Nav2 environment ready!'
            echo 'To run inference:'
            echo '  ros2 run bevnet_nav2 bevnet_inference_node.py --ros-args -p model_path:=/path/to/model.pth'
            echo ''
            echo 'Or use the launch file:'
            echo '  ros2 launch bevnet_nav2 bevnet_inference.launch.py model_path:=/path/to/model.pth'
            exec bash
        fi
    "
#!/bin/bash

# Docker运行脚本 - 使用纯Foxy环境

# 设置默认参数
CONTAINER_NAME="bevnet_nav2_container"
IMAGE_NAME="bevnet_foxy:latest"  # 新的镜像名

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
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
            echo "  --container-name NAME  Container name (default: bevnet_nav2_container)"
            echo "  --image NAME          Docker image name (default: bevnet_foxy:latest)"
            echo "  --help                Show this help message"
            exit 0
            ;;
        *)
            echo "Unknown option: $1"
            echo "Use --help for usage information"
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

# 设置X11权限
xhost +local:docker 2>/dev/null || echo "xhost not available"

# 启动容器
echo "Starting BEVNet Nav2 container..."
echo "Image: $IMAGE_NAME"
echo "Container: $CONTAINER_NAME"
echo "Using ROS Foxy environment"

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
    -e ROS_DISTRO=foxy \
    -e PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v /home/gyc/semantic_bevnet:/workspace/bevnet:rw \
    -v "/media/gyc/Backup Plus7/gyc/thesis":/workspace/data:rw \
    -v /home/gyc/bevnet_nav2_ws:/workspace/bevnet_nav2_ws:rw \
    -v /home/gyc/bevnet_nav2_ws/models:/workspace/models:rw \
    -v /home/gyc/bevnet_nav2_ws/config:/workspace/config:rw \
    -v /home/gyc/bevnet_nav2_ws/scripts:/workspace/scripts:rw \
    --workdir /workspace/bevnet_nav2_ws \
    "$IMAGE_NAME" \
    bash
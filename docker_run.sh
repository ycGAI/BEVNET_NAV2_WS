#!/bin/bash
# 启动BEVNet Nav2 Docker容器（包含ROS2）

# 设置X11权限
xhost +local:docker 2>/dev/null || echo "xhost not available"

# 运行容器
docker run -it --rm \
    --name bevnet_nav2_container \
    --gpus all \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e QT_X11_NO_MITSHM=1 \
    -e PYTHONPATH=/workspace/bevnet:/workspace/bevnet/bevnet:$PYTHONPATH \
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
    -v $HOME/.Xauthority:/root/.Xauthority:rw \
    -v $(pwd)/../semantic_bevnet:/workspace/bevnet:rw \
    -v $(pwd):/workspace/bevnet_nav2_ws:rw \
    bevnet_ros2:latest \
    bash

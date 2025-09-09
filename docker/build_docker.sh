#!/bin/bash

# 构建基于bevnet_modi:v1的新容器

echo "Building new Docker image based on bevnet_modi:v1..."
echo "This image will use ROS Foxy directly without conda"

# 检查Dockerfile是否存在
if [ ! -f "Dockerfile" ]; then
    echo "Error: Dockerfile not found!"
    echo "Please make sure you have the Dockerfile in the current directory"
    exit 1
fi

# 构建镜像
docker build -f Dockerfile -t bevnet_foxy:latest .

if [ $? -eq 0 ]; then
    echo "Successfully built bevnet_foxy:latest"
    echo ""
    echo "You can now run the container using:"
    echo "  ./docker_run.sh"
    echo ""
    echo "Or use docker-compose:"
    echo "  docker-compose up"
else
    echo "Failed to build Docker image"
    exit 1
fi
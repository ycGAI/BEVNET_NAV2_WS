#!/bin/bash

echo "Setting up BEVNet Nav2 environment..."

# Check if ROS2 is sourced
if [ -z "$ROS_DISTRO" ]; then
    echo "Error: ROS2 not sourced. Please source your ROS2 installation first."
    exit 1
fi

# Create workspace structure
WORKSPACE_ROOT=$(pwd)

# Install Python dependencies
echo "Installing Python dependencies..."
pip install -r requirements.txt

# Install additional ROS2 dependencies
echo "Installing ROS2 dependencies..."
sudo apt-get update
sudo apt-get install -y \
    ros-$ROS_DISTRO-navigation2 \
    ros-$ROS_DISTRO-nav2-bringup \
    ros-$ROS_DISTRO-robot-localization \
    ros-$ROS_DISTRO-pcl-ros \
    ros-$ROS_DISTRO-pcl-conversions \
    ros-$ROS_DISTRO-tf2-sensor-msgs \
    ros-$ROS_DISTRO-tf2-geometry-msgs

# Build the workspace
echo "Building workspace..."
colcon build --symlink-install

# Source the workspace
source install/setup.bash

echo "Environment setup complete!"

#!/bin/bash
set -e

# Setup ROS2 environment
source /opt/ros/humble/setup.bash

# Build workspace if needed
if [ ! -d "/bevnet_nav2_ws/install" ]; then
    echo "Building workspace..."
    cd /bevnet_nav2_ws
    colcon build --symlink-install
fi

# Source workspace
source /bevnet_nav2_ws/install/setup.bash

# Execute command
exec "$@"

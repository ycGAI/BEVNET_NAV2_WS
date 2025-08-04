#!/bin/bash

echo "Creating remaining directories and files..."

# 创建顶层目录
mkdir -p models
mkdir -p data/mcap
mkdir -p data/kitti_format
mkdir -p scripts

# 创建资源目录和文件（ROS2包需要）
mkdir -p src/bevnet_nav2_core/resource
mkdir -p src/bevnet_nav2_msgs/resource
mkdir -p src/bevnet_nav2_bringup/resource
mkdir -p src/mcap_replay/resource

touch src/bevnet_nav2_core/resource/bevnet_nav2_core
touch src/bevnet_nav2_msgs/resource/bevnet_nav2_msgs
touch src/bevnet_nav2_bringup/resource/bevnet_nav2_bringup
touch src/mcap_replay/resource/mcap_replay

# 复制脚本文件
cat > scripts/setup_environment.sh << 'EOF'
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
EOF

chmod +x scripts/setup_environment.sh

# 创建requirements.txt
cat > requirements.txt << 'EOF'
# Core dependencies
numpy>=1.24.3
opencv-python>=4.8.0.74
scipy>=1.10.0
pillow>=9.5.0
pyyaml>=6.0

# Deep learning
torch>=2.0.1
torchvision>=0.15.2

# ROS2 related
transforms3d>=0.4.1
pyquaternion>=0.9.9

# MCAP support
mcap>=1.1.1
mcap-ros2-support>=0.5.3

# Visualization
matplotlib>=3.7.1

# Optional but recommended
tqdm>=4.65.0
EOF

# 创建简单的README
cat > README.md << 'EOF'
# BEVNet Nav2 Integration System

This project integrates BEVNet with ROS2 Nav2 for semantic-aware navigation.

## Quick Start

1. Install dependencies:
   ```bash
   ./scripts/setup_environment.sh
   ```

2. Build the workspace:
   ```bash
   colcon build --symlink-install
   source install/setup.bash
   ```

3. Run the system:
   ```bash
   ros2 launch bevnet_nav2_bringup simulation.launch.py
   ```
EOF

# 创建docker entrypoint
cat > docker/entrypoint.sh << 'EOF'
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
EOF

chmod +x docker/entrypoint.sh

echo "Setup complete!"
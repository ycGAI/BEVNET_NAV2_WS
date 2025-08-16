# BEVNet Nav2 Integration System

This project integrates BEVNet with ROS2 Nav2 for semantic-aware navigation.

## Quick Start
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "source /bevnet_nav2_ws/install/setup.bash" >> ~/.bashrc
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

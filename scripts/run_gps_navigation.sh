#!/bin/bash
"""
启动GPS路径跟随导航
使用KITTI的poses.txt作为全局路径
"""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}=== GPS Waypoint Navigation System ===${NC}"
echo ""

# 默认参数
POSES_FILE="/workspace/data/gyc/thesis/rellis_3d/rellis_4class_100x100_2_sl50tr1/sequences/train/poses.txt"
WAYPOINT_SPACING=5.0
GOAL_TOLERANCE=2.0
LOOP=false

# 解析命令行参数
while [[ $# -gt 0 ]]; do
    case $1 in
        --poses)
            POSES_FILE="$2"
            shift 2
            ;;
        --spacing)
            WAYPOINT_SPACING="$2"
            shift 2
            ;;
        --tolerance)
            GOAL_TOLERANCE="$2"
            shift 2
            ;;
        --loop)
            LOOP=true
            shift
            ;;
        --help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --poses FILE      Path to poses.txt file"
            echo "  --spacing DIST    Waypoint spacing in meters (default: 5.0)"
            echo "  --tolerance DIST  Goal tolerance in meters (default: 2.0)"
            echo "  --loop           Enable loop mode"
            echo "  --help           Show this help message"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            exit 1
            ;;
    esac
done

# 检查poses文件
if [ ! -f "$POSES_FILE" ]; then
    echo -e "${RED}Error: Poses file not found: $POSES_FILE${NC}"
    exit 1
fi

echo -e "${GREEN}Configuration:${NC}"
echo "  Poses file: $POSES_FILE"
echo "  Waypoint spacing: ${WAYPOINT_SPACING}m"
echo "  Goal tolerance: ${GOAL_TOLERANCE}m"
echo "  Loop mode: $LOOP"
echo ""

# 设置ROS环境
source /opt/ros/foxy/setup.bash

# 运行GPS路径跟随节点
echo -e "${YELLOW}Starting GPS waypoint follower...${NC}"
python3 /home/claude/gps_waypoint_follower.py \
    --ros-args \
    -p poses_file:="$POSES_FILE" \
    -p waypoint_spacing:=$WAYPOINT_SPACING \
    -p goal_tolerance:=$GOAL_TOLERANCE \
    -p loop:=$LOOP
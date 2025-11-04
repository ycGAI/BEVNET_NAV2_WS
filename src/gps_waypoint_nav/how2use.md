先用/workspace/bevnet_nav2_ws/src/gps_waypoint_nav/gps_waypoint_nav/gps_extractor.py来提取mcap中的gps点
使用/workspace/bevnet_nav2_ws/src/gps_waypoint_nav/gps_waypoint_nav/gps_path_publisher.py来给nav2发布

# 从MCAP文件提取GPS数据
python3 /workspace/bevnet_nav2_ws/install/gps_waypoint_nav/lib/gps_waypoint_nav/extract_gps_from_mcap \
    your_bag.mcap \
    -o /workspace/bevnet_nav2_ws/data/orchard_path \
    -d 1.0  # 1米下采样距离

# 给nav2发布
ros2 run gps_waypoint_nav gps_path_publisher \
    --ros-args \
    -p gps_file:=/workspace/bevnet_nav2_ws/data/orchard_path.json \
    -p use_utm:=true \
    -p frame_id:=map \
    -p publish_markers:=true


# 这个是老的版本， 新的版本，使用kitti数据中pose.txt作为全局路径规划的版本在/workspace/bevnet_nav2_ws/src/gps_waypoint_nav/gps_waypoint_nav/gps_waypoint_follower.py
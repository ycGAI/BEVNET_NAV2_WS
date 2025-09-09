cmake_minimum_required(VERSION 3.5)
project(mcap_replay)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# 安装Python脚本
install(PROGRAMS
  mcap_replay/mcap_player_final.py
  DESTINATION lib/${PROJECT_NAME}
)

# 添加缺少的结尾
ament_package()

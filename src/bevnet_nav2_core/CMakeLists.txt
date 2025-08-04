cmake_minimum_required(VERSION 3.5)
project(bevnet_nav2_core)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python安装
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  bevnet_nav2_core/bevnet_inference_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

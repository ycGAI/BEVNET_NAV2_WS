cmake_minimum_required(VERSION 3.5)
project(bevnet_nav2_core)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

# Find packages
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(bevnet_nav2_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Build only the costmap layer plugin (controller has API issues)
add_library(${PROJECT_NAME} SHARED
  src/bevnet_costmap_layer.cpp
)

# Link libraries
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  nav2_costmap_2d
  nav2_core
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  bevnet_nav2_msgs
)

# Install the library
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Export plugin description
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)

# Python installation
ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
  bevnet_nav2_core/bevnet_inference_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  nav2_costmap_2d
  nav2_core
  pluginlib
  tf2_ros
  tf2_geometry_msgs
  bevnet_nav2_msgs
)

ament_package()

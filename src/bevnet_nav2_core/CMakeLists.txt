cmake_minimum_required(VERSION 3.5)
project(bevnet_nav2_core)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pluginlib REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(bevnet_nav2_msgs REQUIRED)
find_package(angles REQUIRED)

# Include directories
include_directories(
  include
)

# Create bevnet_costmap_layer library
add_library(bevnet_costmap_layer SHARED
  src/bevnet_costmap_layer.cpp
)

# Create bevnet_controller library
add_library(bevnet_controller SHARED
  src/bevnet_controller.cpp
)

# Dependencies for bevnet_costmap_layer
set(bevnet_costmap_layer_dependencies
  rclcpp
  nav2_costmap_2d
  nav_msgs
  sensor_msgs
  geometry_msgs
  tf2_ros
  tf2_geometry_msgs
  bevnet_nav2_msgs
  pluginlib
)

# Dependencies for bevnet_controller
set(bevnet_controller_dependencies
  rclcpp
  nav2_core
  nav2_costmap_2d
  nav2_util
  nav_msgs
  geometry_msgs
  tf2_ros
  pluginlib
  angles
)

# Link dependencies
ament_target_dependencies(bevnet_costmap_layer ${bevnet_costmap_layer_dependencies})
ament_target_dependencies(bevnet_controller ${bevnet_controller_dependencies})

# Register the plugin
pluginlib_export_plugin_description_file(nav2_costmap_2d plugins.xml)
pluginlib_export_plugin_description_file(nav2_core plugins.xml)

# Install libraries
install(TARGETS
  bevnet_costmap_layer
  bevnet_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# Install headers
install(DIRECTORY include/
  DESTINATION include/
)

# Install plugins.xml
install(FILES plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

# Python部分
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  bevnet_nav2_core/bevnet_inference_node.py
  bevnet_nav2_core/costmap_fusion.py
  bevnet_nav2_core/path_visualizer.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(
  bevnet_costmap_layer
  bevnet_controller
)
ament_export_dependencies(${bevnet_costmap_layer_dependencies})
ament_export_dependencies(${bevnet_controller_dependencies})

ament_package()
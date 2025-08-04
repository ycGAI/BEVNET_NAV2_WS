cmake_minimum_required(VERSION 3.5)
project(mcap_replay)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Python部分
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  mcap_replay/mcap_player.py
  mcap_replay/kitti_converter.py
  mcap_replay/data_synchronizer.py
  mcap_replay/mcap_player
  mcap_replay/kitti_converter
  mcap_replay/data_synchronizer
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

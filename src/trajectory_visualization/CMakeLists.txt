cmake_minimum_required(VERSION 3.5)
project(trajectory_visualization)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

# Executable and linking
add_executable(trajectory_visualization src/trajectory_visualization.cpp)
ament_target_dependencies(trajectory_visualization "rclcpp" "sensor_msgs")

# Install
install(TARGETS
  trajectory_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

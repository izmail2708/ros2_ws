cmake_minimum_required(VERSION 3.8)
project(block2_izmailov)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(trajectory_interface REQUIRED)
find_package(abb_irb4600_ikfast REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(trajectory_visualization
  src/trajectory_visualization.cpp
  src/trajectory_visualization_client.cpp)
ament_target_dependencies(trajectory_visualization "rclcpp" "sensor_msgs" "trajectory_interface")

add_executable(ik_trajectory_visualization
  src/ik_trajectory_visualization.cpp
  src/trajectory_visualization_client.cpp)
ament_target_dependencies(ik_trajectory_visualization "rclcpp" "sensor_msgs" "trajectory_interface" "abb_irb4600_ikfast")

add_executable(trajectory_computation
  src/trajectory_computation.cpp
  src/trajectory_visualization_client.cpp)
ament_target_dependencies(trajectory_computation "rclcpp" "sensor_msgs" "trajectory_interface")


install(TARGETS
  trajectory_visualization
  ik_trajectory_visualization
  trajectory_computation
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/draw_graphs.py
  scripts/draw_tool_graphs.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

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

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(trajectory_visualization src/trajectory_visualization.cpp)
ament_target_dependencies(trajectory_visualization "rclcpp" "sensor_msgs")

install(TARGETS
  trajectory_visualization
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/draw_graphs.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()

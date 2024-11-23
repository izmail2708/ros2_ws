#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_interface/srv/compute_trajectory.hpp>

#define FILE_PATH "/home/student/ros2_ws/block2_izmailov/src/ik_data.csv"
#define PARAMETERS_NUMBER 6

double *find_best_solution(ikfast_abb::Solutions solutions,
                           std::vector<double> &previous_solution) {
  if (previous_solution.empty()) {
    double *best_solution = new double[PARAMETERS_NUMBER];
    for (int i = 0; i < PARAMETERS_NUMBER; ++i) {
      best_solution[i] = solutions[2][i];
      previous_solution.push_back(solutions[0][i]);
    }
    return best_solution;
  }

  double *best_solution = new double[PARAMETERS_NUMBER];
  double best_distance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < solutions.size(); ++i) {
    double distance = 0.0;
    for (int j = 0; j < PARAMETERS_NUMBER; ++j) {
      double diff = solutions[i][j] - previous_solution[j];
      distance += diff * diff;
    }

    if (distance < best_distance) {
      best_distance = distance;
      for (int j = 0; j < PARAMETERS_NUMBER; ++j) {
        best_solution[j] = solutions[i][j];
      }
    }
  }

  for (int i = 0; i < PARAMETERS_NUMBER; ++i) {
    previous_solution[i] = best_solution[i];
  }

  return best_solution;
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("trajectory_visualization_client");

  auto client =
      node->create_client<trajectory_interface::srv::ComputeTrajectory>(
          "/trajectory_computation_service");

  if (!client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(),
                 "Service /trajectory_computation_service unavaliable");
    rclcpp::shutdown();
    return 1;
  }

  auto request =
      std::make_shared<trajectory_interface::srv::ComputeTrajectory::Request>();

  std::vector<double> coordinates;

  std::ifstream file(FILE_PATH);
  if (!file.is_open()) {
    RCLCPP_ERROR(node->get_logger(), "Unable to open file: %s", FILE_PATH);
    return 1;
  }

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string value;
    std::vector<double> row;

    while (std::getline(ss, value, ',')) {
      try {
        row.push_back(std::stod(value));
      } catch (const std::exception &e) {
        RCLCPP_ERROR(node->get_logger(), "Failed to parse value: %s",
                     value.c_str());
        return 1;
      }
    }

    if (row.empty() || row.size() < 2) {
      RCLCPP_ERROR(node->get_logger(), "Invalid row format in file.");
      return 1;
    }

    request->timestamps.push_back(row[0]);
    for (size_t i = 1; i < row.size(); ++i) {
      coordinates.push_back(row[i]);
    }
  }
  file.close();

  std::vector<double> previoius_solution;

  for (int i = 0; i < coordinates.size() / PARAMETERS_NUMBER; i++) {
    double x = coordinates[i * PARAMETERS_NUMBER];
    double y = coordinates[i * PARAMETERS_NUMBER + 1];
    double z = coordinates[i * PARAMETERS_NUMBER + 2];
    double rx = coordinates[i * PARAMETERS_NUMBER + 3];
    double ry = coordinates[i * PARAMETERS_NUMBER + 4];
    double rz = coordinates[i * PARAMETERS_NUMBER + 5];

    Eigen::Affine3d pose = Eigen::Translation3d(Eigen::Vector3d(x, y, z)) *
                           Eigen::AngleAxisd(rx, Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(ry, Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(rz, Eigen::Vector3d::UnitZ());

    // Compute IK
    ikfast_abb::Solutions solutions = ikfast_abb::computeIK(pose);
    auto solution = find_best_solution(solutions, previoius_solution);

    for (int i = 0; i < PARAMETERS_NUMBER; i++) {
      request->positions.push_back(solution[i]);
    }
  }

  size_t joint_count = (request->positions.size() / request->timestamps.size());
  request->joint_names.resize(joint_count);
  for (size_t i = 0; i < joint_count; ++i) {
    request->joint_names[i] = "joint" + std::to_string(i + 1);
  }

  request->rows = request->timestamps.size();
  request->cols = joint_count;

  auto future = client->async_send_request(request);

  try {
    auto response = future.get();
    if (response->success) {
      RCLCPP_INFO(node->get_logger(), "Trajectory computation was successful!");
    } else {
      RCLCPP_ERROR(node->get_logger(), "Trajectory computation failed.");
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(node->get_logger(), "Service call failed: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}

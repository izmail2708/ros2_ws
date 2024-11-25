#include "abb_irb4600_ikfast/abb_irb4600_ikfast.h"
#include "block2_izmailov/trajectory_visualization.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <fstream>
#include <map>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_interface/srv/compute_trajectory.hpp>

#define FILE_PATH "/home/student/ros2_ws/block2_izmailov/src/ik_data.csv"
#define PARAMETERS_NUMBER 6

double *findBestSolution(ikfast_abb::Solutions solutions,
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

void computeToolTrajectory(std::vector<double> timestamps,
                           std::vector<double> coordinates) {
  std::map<int, std::string> parameters_names;

  parameters_names[0] = "x";
  parameters_names[1] = "y";
  parameters_names[2] = "z";
  parameters_names[3] = "rx";
  parameters_names[4] = "ry";
  parameters_names[5] = "rz";

  std::vector<std::unique_ptr<std::ofstream>> outFiles;

  for (size_t i = 0; i < PARAMETERS_NUMBER; i++) {
    std::string filePath = RESULT_DIR + parameters_names[i] + ".txt";
    auto outFile = std::make_unique<std::ofstream>(filePath, std::ios::trunc);

    if (!outFile) {
      std::cerr << "Error open: " << filePath << std::endl;
      continue;
    }

    outFiles.push_back(std::move(outFile));
  }

  int frequency = FREQUENCY;                          // Hz
  double period = 1 / static_cast<double>(frequency); // s
  double max_time = timestamps.back();                // s
  int steps = static_cast<int>(max_time / period) + 1;

  std::vector<std::vector<double>> positions(PARAMETERS_NUMBER,
                                             std::vector<double>(steps, 0.0));

  for (size_t i = 1; i < timestamps.size(); i++) {
    for (int j = 0; j < PARAMETERS_NUMBER; j++) {
      auto x = computePolynomialCoefficients(
          N_ORDER, timestamps[i - 1], timestamps[i],
          coordinates[(i - 1) * PARAMETERS_NUMBER + j],
          coordinates[i * PARAMETERS_NUMBER + j]);

      double time = timestamps[i - 1];
      double max_time = timestamps[i];

      while (time <= max_time) {
        positions[j].push_back(getMotionValue(x, time, 0));
        *outFiles[j] << "p:" << time << ":" << getMotionValue(x, time, 0)
                     << std::endl;
        *outFiles[j] << "v:" << time << ":" << getMotionValue(x, time, 1)
                     << std::endl;
        *outFiles[j] << "a:" << time << ":" << getMotionValue(x, time, 2)
                     << std::endl;
        *outFiles[j] << "t:" << time << ":" << getMotionValue(x, time, 3)
                     << std::endl;
        time += period;
      }
    }
  }

  for (auto &file : outFiles) {
    file->close();
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::vector<double> timestamps, positions, coordinates;

  std::ifstream file(FILE_PATH);
  if (!file.is_open()) {
    throw std::runtime_error(std::format("Unable to open file: {}", FILE_PATH));
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
        throw std::runtime_error(
            std::format("Failed to parse value: {}", value.c_str()));
        return 1;
      }
    }

    if (row.empty() || row.size() < 2) {
      throw std::runtime_error("Invalid row format in file.");
      return 1;
    }

    timestamps.push_back(row[0]);
    for (size_t i = 1; i < row.size(); ++i) {
      coordinates.push_back(row[i]);
    }
  }
  file.close();

  computeToolTrajectory(timestamps, coordinates);

  std::vector<double> previoius_solution;

  for (size_t i = 0; i < coordinates.size() / PARAMETERS_NUMBER; i++) {
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
    auto solution = findBestSolution(solutions, previoius_solution);

    for (int i = 0; i < PARAMETERS_NUMBER; i++) {
      positions.push_back(solution[i]);
    }
  }

  try {
    auto node =
        std::make_shared<TrajectoryVisualizationClient>(timestamps, positions);
    rclcpp::spin(node);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception: %s", e.what());
  }

  rclcpp::shutdown();
  return 0;
}

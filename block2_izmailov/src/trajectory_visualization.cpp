#include "block2_izmailov/trajectory_visualization.hpp"
#include <fstream>
#include <sstream>

#define FILE_PATH "/home/student/ros2_ws/block2_izmailov/src/data.csv"

std::pair<std::vector<double>, std::vector<double>>
readDataFromFile(const std::string &file_path) {
  std::ifstream file(file_path);
  if (!file.is_open()) {
    throw std::runtime_error("Unable to open file: " + file_path);
  }

  std::vector<double> timestamps;
  std::vector<double> positions;

  std::string line;
  while (std::getline(file, line)) {
    std::istringstream ss(line);
    std::string value;
    std::vector<double> row;

    while (std::getline(ss, value, ',')) {
      try {
        row.push_back(std::stod(value));
      } catch (const std::exception &e) {
        throw std::runtime_error("Failed to parse value: " + value);
      }
    }

    if (row.empty() || row.size() < 2) {
      throw std::runtime_error("Invalid row format in file.");
    }

    timestamps.push_back(row[0]);
    for (size_t i = 1; i < row.size(); ++i) {
      positions.push_back(row[i]);
    }
  }

  file.close();
  return {timestamps, positions};
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  std::vector<double> timestamps, positions;

  try {
    std::tie(timestamps, positions) = readDataFromFile(FILE_PATH);
  } catch (const std::exception &e) {
    RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Error reading file: %s",
                 e.what());
    rclcpp::shutdown();
    return -1;
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

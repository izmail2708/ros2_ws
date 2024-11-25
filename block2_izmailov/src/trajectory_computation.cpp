#include "block2_izmailov/constants.hpp"
#include "block2_izmailov/trajectory_visualization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_interface/srv/compute_trajectory.hpp"
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <string>

struct Result {
  std::vector<double> timestamps;
  std::vector<double> positions;
};

class TrajectoryComputationService : public rclcpp::Node {
public:
  TrajectoryComputationService() : Node("trajectory_computation_service") {
    service_ =
        this->create_service<trajectory_interface::srv::ComputeTrajectory>(
            "trajectory_computation_service",
            std::bind(&TrajectoryComputationService::my_service_callback, this,
                      std::placeholders::_1, std::placeholders::_2));
  }

  void my_service_callback(
      const std::shared_ptr<
          trajectory_interface::srv::ComputeTrajectory::Request>
          request,
      std::shared_ptr<trajectory_interface::srv::ComputeTrajectory::Response>
          response) {
    RCLCPP_INFO(this->get_logger(),
                "Received request with %d rows and %d columns.", request->rows,
                request->cols);

    size_t expected_size = request->rows * request->cols;
    if (request->positions.size() != expected_size) {
      RCLCPP_ERROR(this->get_logger(),
                   "Invalid positions array size: %zu (expected %zu)",
                   request->positions.size(), expected_size);
      response->success = false;
      return;
    }

    auto data = parseData(request->timestamps, request->joint_names,
                          request->positions, request->rows, request->cols);

    RCLCPP_INFO(this->get_logger(),
                "Starting computation with data and saving results to %s",
                RESULT_DIR);

    Result result = computeTrajectory(data, request->joint_names, RESULT_DIR);

    response->success = true;
    response->timestamps = result.timestamps;
    response->positions = result.positions;
    RCLCPP_INFO(this->get_logger(), "Trajectory computation successful.");
  }

private:
  rclcpp::Service<trajectory_interface::srv::ComputeTrajectory>::SharedPtr
      service_;

  std::vector<ArmData> parseData(const std::vector<double> &timestamps,
                                 const std::vector<std::string> &joint_names,
                                 const std::vector<double> &positions, int rows,
                                 int cols) {
    std::vector<ArmData> res;

    for (int j = 0; j < cols; j++) {
      ArmData entry;
      entry.name = joint_names[j];
      for (int i = 0; i < rows - 1; i++) {
        TimeInterval interval;
        interval.start = timestamps[i];
        interval.end = timestamps[i + 1];
        interval.polynomialCoefficients = computePolynomialCoefficients(
            N_ORDER, interval.start, interval.end, positions[i * cols + j],
            positions[(i + 1) * cols + j]);
        entry.intervals.push_back(interval);
      }

      res.push_back(entry);
    }

    return res;
  }

  std::vector<std::unique_ptr<std::ofstream>>
  openPositionFiles(const std::string &dirPath,
                    const std::vector<std::string> &joint_names) {
    std::vector<std::unique_ptr<std::ofstream>> outFiles;

    for (size_t i = 0; i < joint_names.size(); i++) {
      std::string filePath = dirPath + joint_names[i] + ".txt";
      auto outFile = std::make_unique<std::ofstream>(filePath, std::ios::trunc);

      if (!outFile) {
        std::cerr << "Error open: " << filePath << std::endl;
        continue;
      }

      outFiles.push_back(std::move(outFile));
    }

    return outFiles;
  }

  Result computeTrajectory(std::vector<ArmData> data,
                           std::vector<std::string> &request_joint_names,
                           std::string result_dir_path) {
    Result result;

    int frequency = FREQUENCY;                          // Hz
    double period = 1 / static_cast<double>(frequency); // s
    double max_time = data[0].intervals.back().end;     // s
    double time = 0;
    int steps = static_cast<int>(max_time / period) + 1;

    std::vector<std::vector<double>> positions(6,
                                               std::vector<double>(steps, 0.0));
    const auto positionFiles =
        openPositionFiles(result_dir_path, request_joint_names);

    int counter = 0;
    while (time <= max_time) {
      for (size_t i = 0; i < data.size(); ++i) {
        Eigen::MatrixXd polynomialCoefficients;

        for (const auto &interval : data[i].intervals) {
          if (time >= interval.start && time <= interval.end) {
            polynomialCoefficients = interval.polynomialCoefficients;
          }
        }

        positions[i][counter] = getMotionValue(polynomialCoefficients, time, 0);

        *positionFiles[i] << "p:" << time << ":" << positions[i][counter]
                          << std::endl;
        *positionFiles[i] << "v:" << time << ":"
                          << getMotionValue(polynomialCoefficients, time, 1)
                          << std::endl;
        *positionFiles[i] << "a:" << time << ":"
                          << getMotionValue(polynomialCoefficients, time, 2)
                          << std::endl;
        *positionFiles[i] << "t:" << time << ":"
                          << getMotionValue(polynomialCoefficients, time, 3)
                          << std::endl;
      }

      result.timestamps.push_back(time);
      time += period;
      counter++;
    }

    for (auto &file : positionFiles) {
      file->close();
    }

    for (size_t i = 0; i < data.size(); i++) {
      std::copy(positions[i].begin(), positions[i].end(),
                std::back_inserter(result.positions));
    }

    return result;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv); // Inicializácia ROS2 prostredia
  std::shared_ptr<TrajectoryComputationService> computer =
      std::make_shared<TrajectoryComputationService>();
  rclcpp::spin(computer); // function that blocks the thread and allows the node
                          // to process callbacks
  rclcpp::shutdown();
  return 0; // Ukončenie programu
}

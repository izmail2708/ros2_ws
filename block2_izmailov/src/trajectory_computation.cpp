#include "block2_izmailov/trajectory_visualization.hpp"
#include "rclcpp/rclcpp.hpp"
#include "trajectory_interface/srv/compute_trajectory.hpp"
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <string>
#include <thread>

#define N 6
#define RESULT_DIR "/home/student/ros2_ws/block2_izmailov/results/"

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
                "Starting visualization with data and saving results to %s",
                RESULT_DIR);

    std::thread visualizationThread(
        &TrajectoryComputationService::start_visualization, this, data,
        std::ref(request->joint_names), RESULT_DIR);

    visualizationThread.join();

    response->success = true;
    RCLCPP_INFO(this->get_logger(), "Trajectory computation successful.");
  }

private:
  rclcpp::Service<trajectory_interface::srv::ComputeTrajectory>::SharedPtr
      service_;

  Eigen::MatrixXd createQPolynomial(int n, double t, int capacity) {
    Eigen::MatrixXd res(1, n);
    res.setZero();

    if (capacity < 0) {
      return res;
    }

    for (int i = 0; i < n; i++) {
      if (i >= capacity) {
        res(0, i) = pow(t, i - capacity);
        if (capacity > 0) {
          for (int j = 0; j < capacity; j++) {
            res(0, i) *= (i - j);
          }
        }
      }
    }

    return res;
  }

  Eigen::MatrixXd computePolynomialCoefficients(int n, double t0, double tf,
                                                double qt0, double qtf) {
    Eigen::MatrixXd A(6, 1);
    Eigen::MatrixXd B(6, n);

    A << qt0, 0, 0, qtf, 0, 0;
    B << createQPolynomial(n, t0, 0), createQPolynomial(n, t0, 1),
        createQPolynomial(n, t0, 2), createQPolynomial(n, tf, 0),
        createQPolynomial(n, tf, 1), createQPolynomial(n, tf, 2);

    Eigen::MatrixXd C = B.colPivHouseholderQr().solve(A);
    return C;
  }

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
            N, interval.start, interval.end, positions[i * cols + j],
            positions[(i + 1) * cols + j]);
        entry.intervals.push_back(interval);
      }

      res.push_back(entry);
    }

    return res;
  }

  double computePosition(const std::vector<ArmData> &data, const int &armNumber,
                         const double &time) {
    if (data.size() < armNumber) {
      return 0.0;
    }

    double position = 0;

    for (const auto &interval : data[armNumber].intervals) {
      if (time >= interval.start && time <= interval.end) {
        for (int i = 0; i < interval.polynomialCoefficients.size(); i++) {
          position += pow(time, i) * interval.polynomialCoefficients(i);
        }
      }
    }

    return position;
  }

  double computeVelocity(const std::vector<ArmData> &data, const int &armNumber,
                         const double &time) {
    if (data.size() < armNumber) {
      return 0.0;
    }

    double velocity = 0;

    for (const auto &interval : data[armNumber].intervals) {
      if (time >= interval.start && time <= interval.end) {
        for (int i = 1; i < interval.polynomialCoefficients.size(); i++) {
          velocity += i * pow(time, i - 1) * interval.polynomialCoefficients(i);
        }
      }
    }

    return velocity;
  }

  double computeAcceleration(const std::vector<ArmData> &data,
                             const int &armNumber, const double &time) {
    if (data.size() < armNumber) {
      return 0.0;
    }

    double acceleration = 0;

    for (const auto &interval : data[armNumber].intervals) {
      if (time >= interval.start && time <= interval.end) {
        for (int i = 2; i < interval.polynomialCoefficients.size(); i++) {
          acceleration += (i - 1) * i * pow(time, i - 2) *
                          interval.polynomialCoefficients(i);
        }
      }
    }

    return acceleration;
  }

  std::vector<std::unique_ptr<std::ofstream>>
  openPositionFiles(const std::string &dirPath,
                    const std::vector<std::string> &joint_names) {
    std::vector<std::unique_ptr<std::ofstream>> outFiles;

    for (int i = 0; i < joint_names.size(); i++) {
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

  void start_visualization(std::vector<ArmData> data,
                           std::vector<std::string> &request_joint_names,
                           std::string result_dir_path) {
    // Create a publisher for joint states
    auto publisher = this->create_publisher<sensor_msgs::msg::JointState>(
        "joint_states", 10);

    // Set joint names
    std::vector<std::string> joint_names = {"joint_1", "joint_2", "joint_3",
                                            "joint_4", "joint_5", "joint_6"};

    // Initialize joint state message
    sensor_msgs::msg::JointState joint_state;
    joint_state.name = joint_names;
    joint_state.position.resize(joint_names.size(), 0.0);
    joint_state.velocity.resize(joint_names.size(), 0.0);
    joint_state.effort.resize(joint_names.size(), 0.0);

    int frequency = 100;                                // Hz
    double period = 1 / static_cast<double>(frequency); // s
    double max_time = 4.5;                              // s
    double time = 0.0;
    bool position_are_written = false;

    const auto positionFiles =
        openPositionFiles(result_dir_path, request_joint_names);

    rclcpp::Rate loop_rate(frequency); // publishing rate

    // Loop to create a trajectory where each joint has value q(t) = t * factor
    while (rclcpp::ok()) {
      for (size_t i = 0; i < 3; ++i) {
        joint_state.position[i] = computePosition(data, i, time);

        if (!position_are_written) {
          *positionFiles[i] << "p:" << time << ":" << joint_state.position[i]
                            << std::endl;
          *positionFiles[i] << "v:" << time << ":"
                            << computeVelocity(data, i, time) << std::endl;
          *positionFiles[i] << "a:" << time << ":"
                            << computeAcceleration(data, i, time) << std::endl;
        }
      }

      // Set the current time for the joint state
      joint_state.header.stamp = this->get_clock()->now();

      // Publish the joint state
      publisher->publish(joint_state);

      // Update time and sleep
      time += period;

      // Loop time
      if (time > max_time) {
        time = 0.0;

        if (!position_are_written) {
          for (auto &file : positionFiles) {
            file->close();
          }

          position_are_written = true;
        }
      }

      loop_rate.sleep();
    }
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

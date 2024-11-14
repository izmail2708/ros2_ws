#include <Eigen/Geometry>
#include <cmath>
#include <cstddef>
#include <fstream>
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sstream>
#include <string>
#include <unordered_set>
#include <vector>

#define N 4
#define DATA_PATH "/home/student/ros2_ws/block2_izmailov/src/data.csv"
#define DIR_PATH "/home/student/ros2_ws/block2_izmailov/results/"

struct TimeInterval {
  double start;
  double end;

  Eigen::MatrixXd polynomialCoefficients;
};

struct ArmData {
  std::vector<TimeInterval> intervals;
};

Eigen::MatrixXd createQPolynomial(int n, double t) {
  Eigen::MatrixXd res(1, n);

  for (int i = 0; i < n; i++) {
    res(0, i) = pow(t, i);
  }

  return res;
}

Eigen::MatrixXd createQDerivalPolynomial(int n, double t) {
  Eigen::MatrixXd res(1, n);

  for (int i = 0; i < n; i++) {
    if (i == 0) {
      res(0, i) = 0;
    } else {
      res(0, i) = i * pow(t, i - 1);
    }
  }

  return res;
}

Eigen::MatrixXd computePolynomialCoefficients(int n, double t0, double tf,
                                              double qt0, double qt0_dot,
                                              double qtf, double qtf_dot) {
  Eigen::MatrixXd A(4, 1);
  Eigen::MatrixXd B(4, n);

  A << qt0, qt0_dot, qtf, qtf_dot;
  B << createQPolynomial(n, t0), createQDerivalPolynomial(n, t0),
      createQPolynomial(n, tf), createQDerivalPolynomial(n, tf);

  Eigen::MatrixXd C = B.colPivHouseholderQr().solve(A);
  return C;
}

std::vector<ArmData> parseData(const std::string &file_path, int rows,
                               int cols) {
  std::vector<ArmData> res;
  std::ifstream file(file_path);
  if (!file.is_open()) {
    std::cerr << "Chyba: " << file_path << std::endl;
    return res;
  }
  std::string line;

  int row = 0;
  double data[rows][cols];

  while (std::getline(file, line) && row < rows) {
    std::stringstream lineStream(line);
    std::string cell;
    int col = 0;

    while (std::getline(lineStream, cell, ',') && col < cols) {
      double value = std::stod(cell);
      data[row][col] = value;
      col++;
    }

    row++;
  }

  for (int j = 1; j < cols; j++) {
    ArmData entry;
    for (int i = 1; i < rows; i++) {
      TimeInterval interval;
      interval.start = data[i - 1][0];
      interval.end = data[i][0];
      interval.polynomialCoefficients = computePolynomialCoefficients(
          N, interval.start, interval.end, data[i - 1][j], 0, data[i][j], 0);
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
        acceleration +=
            (i - 1) * i * pow(time, i - 2) * interval.polynomialCoefficients(i);
      }
    }
  }

  return acceleration;
}

std::vector<std::unique_ptr<std::ofstream>>
openPositionFiles(const std::string &dirPath, const int &armsNumber) {
  std::vector<std::unique_ptr<std::ofstream>> outFiles;

  for (int i = 0; i < armsNumber; i++) {
    std::string filePath = dirPath + "klb_" + std::to_string(i) + ".txt";
    auto outFile = std::make_unique<std::ofstream>(filePath, std::ios::app);

    if (!outFile) {
      std::cerr << "Error open: " << filePath << std::endl;
      continue;
    }

    outFiles.push_back(std::move(outFile));
  }

  return outFiles;
}

int main(int argc, char **argv) {
  auto data = parseData(DATA_PATH, 3, 4);

  // Initialize ROS2 node
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("trajectory_visualization");

  // Create a publisher for joint states
  auto publisher =
      node->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);

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

  const auto positionFiles = openPositionFiles(DIR_PATH, 3);

  rclcpp::Rate loop_rate(frequency); // publishing rate

  // Loop to create a trajectory where each joint has value q(t) = t * factor
  while (rclcpp::ok()) {
    for (size_t i = 0; i < 3; ++i) {
      joint_state.position[i] = computePosition(data, i, time);

      *positionFiles[i] << "p:" << time << ":" << joint_state.position[i]
                        << std::endl;
      *positionFiles[i] << "v:" << time << ":" << computeVelocity(data, i, time)
                        << std::endl;
      *positionFiles[i] << "a:" << time << ":"
                        << computeAcceleration(data, i, time) << std::endl;
    }

    // Set the current time for the joint state
    joint_state.header.stamp = node->get_clock()->now();

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

  // Shutdown ROS2
  rclcpp::shutdown();
  return 0;
}

#include <fstream>
#include <rclcpp/rclcpp.hpp>
#include <trajectory_interface/srv/compute_trajectory.hpp>

#define FILE_PATH "/home/student/ros2_ws/block2_izmailov/src/data.csv"

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
      request->positions.push_back(row[i]);
    }
  }

  file.close();

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

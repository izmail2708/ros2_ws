#include "cstdio"
#include "exception"
#include "izmailov_interface/srv/save_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/srv/command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <rclcpp/client.hpp>
#include <stdexcept>
#include <system_error>

using namespace std;

class Teleop : public rclcpp::Node {
public:
  Teleop() : Node("Teleop") {
    RCLCPP_INFO(this->get_logger(), "Teleop initialized");
    robot_client_ = this->create_client<rrm_msgs::srv::Command>("move_command");
    save_state_client_ =
        this->create_client<izmailov_interface::srv::SaveState>("save_state");

    positions_ = {0.0, 0.0, 0.0};
    send_request_save_state(positions_, 0);
    point_cnt_ = 1;
  }

  void move(const std::vector<double> &positions, double max_velocity) {
    if (move_(positions,
              calculate_joint_speeds(positions_, positions, max_velocity))) {
      send_request_save_state(positions, max_velocity);
    }
  }

  void connect() {
    if (!robot_client_->wait_for_service(std::chrono::seconds(10))) {
      throw std::runtime_error("Robot service not available...");
    }
    if (!save_state_client_->wait_for_service(std::chrono::seconds(10))) {
      throw std::runtime_error("Save state service not available...");
    }
  }

private:
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr robot_client_;
  rclcpp::Client<izmailov_interface::srv::SaveState>::SharedPtr
      save_state_client_;
  std::vector<double> positions_;
  int point_cnt_;

  std::vector<double>
  calculate_joint_speeds(const std::vector<double> &current_positions,
                         const std::vector<double> &target_positions,
                         double max_speed) {
    std::vector<double> speeds(current_positions.size(), 0.0);

    double min_distance = std::numeric_limits<double>::max();
    for (size_t i = 0; i < current_positions.size(); ++i) {
      double distance = std::abs(target_positions[i] - current_positions[i]);
      min_distance = std::min(min_distance, distance);
    }

    double time = min_distance / max_speed;

    for (size_t i = 0; i < current_positions.size(); ++i) {
      double distance = target_positions[i] - current_positions[i];
      speeds[i] = distance / time;
    }

    return speeds;
  }

  void send_request_save_state(const std::vector<double> &joint_angles,
                               double max_speed) {
    if (!save_state_client_->wait_for_service(std::chrono::seconds(5))) {
      RCLCPP_ERROR(this->get_logger(), "Service is not available.");
      return;
    }

    auto request =
        std::make_shared<izmailov_interface::srv::SaveState::Request>();
    request->point_id = point_cnt_++;
    request->joint_angles = joint_angles;
    request->max_speed = max_speed;

    auto future = save_state_client_->async_send_request(request);
    auto result = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future, std::chrono::seconds(5));

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Save sate service call succeeded");
    } else {
      RCLCPP_ERROR(this->get_logger(), "Save state service call failed");
    }
  }

  bool move_(const std::vector<double> &positions,
             const std::vector<double> &velocities) {
    auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
    request->positions = positions;
    std::vector<double> current_positions = positions_;
    request->velocities = velocities;

    positions_ = positions;
    auto future = robot_client_->async_send_request(request);

    auto result = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future, std::chrono::seconds(5));

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s",
                  future.get()->message.c_str());
      return true;

    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
      return false;
    }
  }
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  Teleop robot;

  try {
    robot.connect();
  } catch (const std::system_error &e) {
    std::cerr << e.what();
    return 1;
  }

  robot.move({0.4, -0.2, 1}, 0.1);
  robot.move({0.4, 0.2, 1}, 0.1);

  return 0;
}

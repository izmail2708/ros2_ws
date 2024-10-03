#include "cstdio"
#include "exception"
#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/srv/command.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <stdexcept>
#include <system_error>

using namespace std;

class Teleop : public rclcpp::Node {
public:
  Teleop() : Node("Teleop") {
    RCLCPP_INFO(this->get_logger(), "Teleop initialized");
    client_ = this->create_client<rrm_msgs::srv::Command>("move_command");

    positions_ = {0.0, 0.0, 0.0};
  }

  bool move(const std::vector<double> &positions, double max_velocity) {
    auto request = std::make_shared<rrm_msgs::srv::Command::Request>();
    request->positions = positions;
    std::vector<double> current_positions = positions_;
    request->velocities =
        calculate_joint_speeds(current_positions, positions, max_velocity);

    positions_ = positions;
    auto future = client_->async_send_request(request);

    auto result = rclcpp::spin_until_future_complete(
        this->get_node_base_interface(), future, std::chrono::seconds(5));

    if (result == rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_INFO(this->get_logger(), "Service call succeeded: %s",
                  future.get()->message.c_str());
    } else {
      RCLCPP_ERROR(this->get_logger(), "Service call failed");
    }

    return true;
  }

  void connect() {
    if (!client_->wait_for_service(std::chrono::seconds(10))) {
      throw std::runtime_error("Service not available...");
    }
  }

private:
  rclcpp::Client<rrm_msgs::srv::Command>::SharedPtr client_;
  std::vector<double> positions_;

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

    for (double item : speeds) {
      printf("%f\n", item);
    }

    return speeds;
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

  return 0;
}

#include "cstdio"
#include "iostream"
#include "rclcpp/rclcpp.hpp"
#include "rrm_msgs/msg/command.hpp"
#include "rrm_msgs/srv/command.hpp"

using namespace std;

class Teleop : public rclcpp::Node {
public:
  Teleop() : Node("Teleop") {
    RCLCPP_INFO(this->get_logger(), "Teleop initialized");
    publisher_ =
        this->create_publisher<rrm_msgs::msg::Command>("move_command", 10);
  }
  void move(int joint_id, double position) {
    rrm_msgs::msg::Command message;
    message.joint_id = joint_id;
    message.position = position;
    publisher_->publish(message);
  }

private:
  rclcpp::Publisher<rrm_msgs::msg::Command>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  Teleop robot;

  while (true) {
    int joint_id;
    double position;
    cout << "zadajte klb:\n";
    cin >> joint_id;

    if (joint_id == -1) {
      break;
    }

    cout << "zadajte polohu:\n";
    cin >> position;

    robot.move(joint_id, position);
  }

  return 0;
}

#include "block1_izmailov/save_state.hpp"
#include "izmailov_interface/srv/save_state.hpp"
#include "rclcpp/rclcpp.hpp"
#include <memory>
#include <string>
#include <vector>

class JointLogger : public rclcpp::Node {
public:
  JointLogger() : Node("save_robot_state_service") {
    service_ = this->create_service<izmailov_interface::srv::SaveState>(
        "save_state", std::bind(&JointLogger::save_state_callback, this,
                                std::placeholders::_1, std::placeholders::_2));
  }
  void save_state_callback(
      const std::shared_ptr<izmailov_interface::srv::SaveState::Request>
          request,
      std::shared_ptr<izmailov_interface::srv::SaveState::Response> response);

private:
  rclcpp::Service<izmailov_interface::srv::SaveState>::SharedPtr service_;
};

void JointLogger::save_state_callback(
    const std::shared_ptr<izmailov_interface::srv::SaveState::Request> request,
    std::shared_ptr<izmailov_interface::srv::SaveState::Response> response) {
  int point_id = request->point_id;
  std::vector<double> joint_angles = request->joint_angles;
  double max_speed = request->max_speed;

  response->success = save_robot_state("/home/student/ros2_ws/file.csv",
                                       point_id, joint_angles, max_speed);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<JointLogger> logger = std::make_shared<JointLogger>();
  rclcpp::spin(logger); // function that blocks the thread and allows the node
                        // to process callbacks
  rclcpp::shutdown();
  return 0;
}

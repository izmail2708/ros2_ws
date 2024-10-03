#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
// header file pre msg interface
class JointLogger : public rclcpp::Node {
public:
  JointLogger();
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
};

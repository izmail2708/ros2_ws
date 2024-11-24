#include "block2_izmailov/constants.hpp"
#include <Eigen/Geometry>
#include <cstddef>
#include <rclcpp/rclcpp.hpp>
#include <rcutils/error_handling.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <string>
#include <trajectory_interface/srv/compute_trajectory.hpp>
#include <vector>

struct TimeInterval {
  double start;
  double end;

  Eigen::MatrixXd polynomialCoefficients;
};

struct ArmData {
  std::string name;

  std::vector<TimeInterval> intervals;
};

class TrajectoryVisualizationClient : public rclcpp::Node {
public:
  TrajectoryVisualizationClient(const std::vector<double> &timestamps,
                                const std::vector<double> &positions);

private:
  rclcpp::Client<trajectory_interface::srv::ComputeTrajectory>::SharedPtr
      client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  std::vector<std::string> joint_names;

  void sendRequest(std::vector<double> timestamps,
                   std::vector<double> positions);
  void initializePosition();
  void startVisualization(std::vector<double> timestamps,
                          std::vector<double> positions, size_t joint_count);
};

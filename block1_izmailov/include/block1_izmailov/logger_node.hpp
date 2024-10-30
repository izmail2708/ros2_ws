#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <numbers>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

class DHParameter {
public:
  double a;
  double alpha;
  double d;
  double theta;

  DHParameter(double a, double alpha, double d, double theta);
};

class JointLogger : public rclcpp::Node {
public:
  JointLogger();
  void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);

private:
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
  Eigen::MatrixXd get_a_matrix_from_dh(double a, double alpha, double d,
                                       double theta);
  void fill_a_matrices();
  std::vector<DHParameter> dh_parameters;
  std::vector<Eigen::MatrixXd> a_matrices;
  tf2_ros::TransformBroadcaster tf_broadcaster_;
  void publishTransform(const Eigen::Matrix4d &matrix);
};

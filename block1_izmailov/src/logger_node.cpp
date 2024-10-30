#include "block1_izmailov/logger_node.hpp"
#include "Eigen/src/Core/Matrix.h"
#include "rclcpp/rclcpp.hpp"
#include <rclcpp/executors/multi_threaded_executor.hpp>

using namespace std;

DHParameter::DHParameter(double a, double alpha, double d, double theta)
    : a(a), alpha(alpha), d(d), theta(theta) {}

Eigen::MatrixXd JointLogger::get_a_matrix_from_dh(double a, double alpha,
                                                  double d, double theta) {
  Eigen::MatrixXd res(4, 4);
  res << cos(theta), -sin(theta) * cos(alpha), sin(theta) * sin(alpha),
      a * cos(theta), sin(theta), cos(theta) * cos(alpha),
      -cos(theta) * sin(alpha), a * sin(theta), 0, sin(alpha), cos(alpha), d, 0,
      0, 0, 1;

  return res;
}

void JointLogger::fill_a_matrices() {
  a_matrices.resize(size(dh_parameters));

  for (size_t i = 0; i < dh_parameters.size(); ++i) {
    a_matrices[i] =
        get_a_matrix_from_dh(dh_parameters[i].a, dh_parameters[i].alpha,
                             dh_parameters[i].d, dh_parameters[i].theta);
  }
}

void JointLogger::publishTransform(const Eigen::Matrix4d &matrix) {
  geometry_msgs::msg::TransformStamped transform;

  transform.header.stamp = this->get_clock()->now();
  transform.header.frame_id = "base_link";
  transform.child_frame_id = "tool0_calculated";

  transform.transform.translation.x = matrix(0, 3);
  transform.transform.translation.y = matrix(1, 3);
  transform.transform.translation.z = matrix(2, 3);

  Eigen::Quaterniond q(matrix.block<3, 3>(0, 0)); // Извлечение вращения
  transform.transform.rotation.x = q.x();
  transform.transform.rotation.y = q.y();
  transform.transform.rotation.z = q.z();
  transform.transform.rotation.w = q.w();

  tf_broadcaster_.sendTransform(transform);
}

JointLogger::JointLogger() : Node("joint_logger"), tf_broadcaster_(this) {
  subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "joint_states", 10,
      std::bind(&JointLogger::joint_states_callback, this,
                std::placeholders::_1));

  dh_parameters = {{0, numbers::pi / 2, 0, numbers::pi},
                   {0.203, 0, 0, numbers::pi / 2},
                   {0, numbers::pi / 2, 0, numbers::pi / 2},
                   {0, numbers::pi / 2, 0.406, numbers::pi},
                   {0, numbers::pi / 2, 0, numbers::pi},
                   {0, 0, 0.2, 0}};

  fill_a_matrices();
}

void JointLogger::joint_states_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg) {
  dh_parameters[0].theta = numbers::pi + msg->position[0];
  dh_parameters[1].theta = numbers::pi / 2 + msg->position[1];
  dh_parameters[2].theta = numbers::pi / 2 + msg->position[2];
  dh_parameters[3].theta = numbers::pi + msg->position[3];
  dh_parameters[4].theta = numbers::pi + msg->position[4];
  dh_parameters[5].d = 0.2 + msg->position[5];
  fill_a_matrices();
  if (!a_matrices.empty()) {
    auto tool_matrix = a_matrices[0];

    for (size_t i = 1; i < 6; ++i) {
      tool_matrix *= a_matrices[i];
    }

    cout << "tool_matrix:\n" << tool_matrix << endl;

    publishTransform(tool_matrix);
  }
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  std::shared_ptr<JointLogger> logger = std::make_shared<JointLogger>();
  rclcpp::spin(logger);
  rclcpp::shutdown();
  return 0;
}

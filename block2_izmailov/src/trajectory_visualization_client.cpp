#include "block2_izmailov/trajectory_visualization.hpp"

TrajectoryVisualizationClient::TrajectoryVisualizationClient(
    const std::vector<double> &timestamps, const std::vector<double> &positions)
    : Node("trajectory_visualization_client"),
      client_(this->create_client<trajectory_interface::srv::ComputeTrajectory>(
          "/trajectory_computation_service")),
      publisher_(this->create_publisher<sensor_msgs::msg::JointState>(
          "joint_states", 10)),
      joint_names(
          {"joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"}) {

  if (!client_->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(this->get_logger(),
                 "Service /trajectory_computation_service unavailable");
    rclcpp::shutdown();
    throw std::runtime_error(
        "Service /trajectory_computation_service unavailable");
  }

  initializePosition();
  sendRequest(timestamps, positions);
}

void TrajectoryVisualizationClient::sendRequest(std::vector<double> timestamps,
                                                std::vector<double> positions) {
  auto request =
      std::make_shared<trajectory_interface::srv::ComputeTrajectory::Request>();
  request->timestamps = timestamps;
  request->positions = positions;

  size_t joint_count = (request->positions.size() / request->timestamps.size());
  request->joint_names.resize(joint_count);
  for (size_t i = 0; i < joint_count; ++i) {
    request->joint_names[i] = "joint" + std::to_string(i + 1);
  }

  request->rows = request->timestamps.size();
  request->cols = joint_count;

  auto future = client_->async_send_request(request);

  if (rclcpp::spin_until_future_complete(this->get_node_base_interface(),
                                         future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();

    if (response->success) {
      RCLCPP_INFO(this->get_logger(), "Response: success");
      startVisualization(response->timestamps, response->positions,
                         request->cols);
    } else {
      RCLCPP_INFO(this->get_logger(), "Response: fail");
    }
  } else {
    RCLCPP_ERROR(this->get_logger(), "Failed to call service");
  }
}

void TrajectoryVisualizationClient::initializePosition() {
  // Initialize joint state message
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names;
  joint_state.position.resize(joint_names.size(), 0.0);

  for (size_t i = 0; i < joint_state.position.size(); ++i) {
    joint_state.position[i] = 0;
  }

  // Set the current time for the joint state
  joint_state.header.stamp = this->get_clock()->now();

  // Publish the joint state
  publisher_->publish(joint_state);

  RCLCPP_INFO(this->get_logger(), "The robot's positions are initialized.");
}

void TrajectoryVisualizationClient::startVisualization(
    std::vector<double> timestamps, std::vector<double> positions,
    size_t joint_count) {
  sensor_msgs::msg::JointState joint_state;
  joint_state.name = joint_names;
  joint_state.position.resize(joint_names.size(), 0.0);

  int frequency = FREQUENCY;                          // Hz
  double period = 1 / static_cast<double>(frequency); // s
  double max_time = timestamps.back();                // s
  double time = 0.0;
  int steps = static_cast<int>(max_time / period) + 1;

  rclcpp::Rate loop_rate(frequency); // publishing rate

  int counter = 0;
  while (rclcpp::ok()) {
    for (size_t i = 0; i < joint_count; ++i) {
      joint_state.position[i] = positions[i * steps + counter];
    }

    // Set the current time for the joint state
    joint_state.header.stamp = this->get_clock()->now();

    // Publish the joint state
    publisher_->publish(joint_state);

    // Update time and sleep
    time += period;
    counter++;

    // Loop time
    if (time > max_time) {
      time = 0.0;
      counter = 0;
    }

    loop_rate.sleep();
  }
}

Eigen::MatrixXd createQPolynomial(int n, double t, int order) {
  Eigen::MatrixXd res(1, n);
  res.setZero();

  if (order < 0) {
    return res;
  }

  for (int i = 0; i < n; i++) {
    if (i >= order) {
      res(0, i) = pow(t, i - order);
      if (order > 0) {
        for (int j = 0; j < order; j++) {
          res(0, i) *= (i - j);
        }
      }
    }
  }

  return res;
}

Eigen::MatrixXd computePolynomialCoefficients(int n, double t0, double tf,
                                              double qt0, double qtf) {
  Eigen::MatrixXd A(6, 1);
  Eigen::MatrixXd B(6, n);

  A << qt0, 0, 0, qtf, 0, 0;
  B << createQPolynomial(n, t0, 0), createQPolynomial(n, t0, 1),
      createQPolynomial(n, t0, 2), createQPolynomial(n, tf, 0),
      createQPolynomial(n, tf, 1), createQPolynomial(n, tf, 2);

  Eigen::MatrixXd C = B.colPivHouseholderQr().solve(A);
  return C;
}

double getMotionValue(Eigen::MatrixXd polynomialCoefficients,
                      const double &time, int order) {
  double position = 0;

  for (int i = order; i < polynomialCoefficients.size(); i++) {
    double temp = polynomialCoefficients(i);
    temp *= pow(time, i - order);
    if (order > 0) {
      for (int j = 0; j < order; j++) {
        temp *= (i - j);
      }
    }
    position += temp;
  }

  return position;
}

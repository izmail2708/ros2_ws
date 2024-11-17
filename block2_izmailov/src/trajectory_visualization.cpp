#include <rclcpp/rclcpp.hpp>
#include <trajectory_interface/srv/compute_trajectory.hpp>

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
  request->datafile_path = "/home/student/ros2_ws/block2_izmailov/src/data.csv";
  request->result_dir_path = "/home/student/ros2_ws/block2_izmailov/results/";

  auto future = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, future) ==
      rclcpp::FutureReturnCode::SUCCESS) {
    auto response = future.get();
    RCLCPP_INFO(node->get_logger(), "Operation success: %s",
                response->success ? "true" : "false");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Error calling the service");
  }

  rclcpp::shutdown();
  return 0;
}

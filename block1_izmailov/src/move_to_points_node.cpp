#include "izmailov_interface/srv/move_to_points.hpp"
#include "rclcpp/rclcpp.hpp"
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

class MoveToPointsServer : public rclcpp::Node {
public:
  MoveToPointsServer() : Node("move_to_points_server") {
    service_ = this->create_service<izmailov_interface::srv::MoveToPoints>(
        "move_to_points",
        std::bind(&MoveToPointsServer::handle_move_to_points, this,
                  std::placeholders::_1, std::placeholders::_2));
  }

private:
  void handle_move_to_points(
      const std::shared_ptr<izmailov_interface::srv::MoveToPoints::Request>
          request,
      std::shared_ptr<izmailov_interface::srv::MoveToPoints::Response>
          response) {
    RCLCPP_INFO(this->get_logger(), "Načítavam body zo súboru...");

    std::string file_path = request->file_path;

    std::vector<int32_t> point_ids;
    std::vector<double> joint_angles;
    std::vector<double> max_speeds;

    bool success =
        load_points_from_file(file_path, point_ids, joint_angles, max_speeds);

    if (!success) {
      RCLCPP_ERROR(this->get_logger(), "Error reading the file.");
      return;
    }

    response->point_id = point_ids;
    response->joint_angles = joint_angles;
    response->max_speed = max_speeds;
    response->success = true;

    RCLCPP_INFO(this->get_logger(), "Data successfully sent.");
  }

  bool load_points_from_file(const std::string &file_path,
                             std::vector<int32_t> &point_ids,
                             std::vector<double> &joint_angles,
                             std::vector<double> &max_speeds) {
    std::ifstream file(file_path);

    if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Could not open file: %s",
                   file_path.c_str());
      return false;
    }

    std::string line;
    while (std::getline(file, line)) {
      std::stringstream ss(line);
      std::string value;

      int32_t point_id;
      double max_speed;
      std::vector<double> angles(3);

      std::getline(ss, value, ',');
      try {
        point_id = std::stoi(value);
      } catch (const std::invalid_argument &e) {
        RCLCPP_ERROR(this->get_logger(), "Error reading point_id: %s",
                     e.what());
        return false;
      }

      for (int i = 0; i < 3; ++i) {
        if (!std::getline(ss, value, ',')) {
          RCLCPP_ERROR(this->get_logger(), "Not enough data for joint_angles.");
          return false;
        }
        try {
          angles[i] = std::stod(value);
        } catch (const std::invalid_argument &e) {
          RCLCPP_ERROR(this->get_logger(), "Error reading joint_angle: %s",
                       e.what());
          return false;
        }
      }

      if (std::getline(ss, value, ',')) {
        try {
          max_speed = std::stod(value);
        } catch (const std::invalid_argument &e) {
          RCLCPP_ERROR(this->get_logger(), "Error reading max_speed: %s",
                       e.what());
          return false;
        }
      } else {
        RCLCPP_ERROR(this->get_logger(), "Error: could not read max_speed.");
        return false;
      }

      point_ids.push_back(point_id);
      joint_angles.insert(joint_angles.end(), angles.begin(), angles.end());
      max_speeds.push_back(max_speed);
    }

    file.close();
    RCLCPP_INFO(this->get_logger(), "Successfully loaded %lu points",
                point_ids.size());
    return true;
  }

  rclcpp::Service<izmailov_interface::srv::MoveToPoints>::SharedPtr service_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MoveToPointsServer>());
  rclcpp::shutdown();
  return 0;
}

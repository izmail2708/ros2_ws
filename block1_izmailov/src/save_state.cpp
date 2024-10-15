#include "block1_izmailov/save_state.hpp"
#include <fstream>
#include <iostream>

bool save_robot_state(const std::string &file_path, int point_id,
                      const std::vector<double> &joint_angles,
                      double max_speed) {
  std::ofstream file(file_path, std::ios::app);

  if (file.is_open()) {
    file << point_id;
    for (const auto &angle : joint_angles) {
      file << "," << angle;
    }
    file << "," << max_speed << std::endl;
    file.close();
    return true;
  } else {
    std::cerr << "Error while writing in file" << std::endl;
    return false;
  }
}

#ifndef SAVE_STATE_HPP
#define SAVE_STATE_HPP

#include <string>
#include <vector>

bool save_robot_state(const std::string &file_path, int point_id,
                      const std::vector<double> &joint_angles,
                      double max_speed);

#endif

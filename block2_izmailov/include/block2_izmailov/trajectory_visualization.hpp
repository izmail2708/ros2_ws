#include <Eigen/Geometry>
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

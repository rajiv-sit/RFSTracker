#pragma once

#include <Eigen/Dense>

namespace rfs {

/** @brief Indexed target state consumed by trackers and sensors. */
struct TargetState_t {
  int id = -1;
  Eigen::Vector4d state = Eigen::Vector4d::Zero(); // [x, y, vx, vy]
  double lastManeuverTime = 0.0;
  double birthTime = 0.0;
  double deathTime = 0.0;
};

} // namespace rfs

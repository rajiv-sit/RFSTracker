#pragma once

#include <Eigen/Dense>

namespace rfs {

/** @brief Defines target dynamics and optional Jacobian support. */
class MotionModel {
public:
  virtual ~MotionModel() = default;

  /** @brief Predict next state using state vector and dt. */
  virtual Eigen::VectorXd propagate(const Eigen::VectorXd &state, double dt) const = 0;

  /** @brief Optional linearization of motion dynamics. */
  virtual Eigen::MatrixXd jacobian(const Eigen::VectorXd &state, double dt) const {
    return Eigen::MatrixXd();
  }
};

} // namespace rfs

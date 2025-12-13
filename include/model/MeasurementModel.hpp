#pragma once

#include <Eigen/Dense>

namespace rfs {

/** @brief Sensor measurement function abstraction. */
class MeasurementModel {
public:
  virtual ~MeasurementModel() = default;

  /** @brief Map state to measurement space. */
  virtual Eigen::VectorXd measure(const Eigen::VectorXd &state) const = 0;

  /** @brief Optionally provide Jacobian for the measurement. */
  virtual Eigen::MatrixXd jacobian(const Eigen::VectorXd &state) const {
    return Eigen::MatrixXd();
  }
};

} // namespace rfs

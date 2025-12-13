#pragma once

#include <Eigen/Dense>

namespace rfs {

/** @brief Gate measurements against predicted states. */
class GatingPolicy {
public:
  virtual ~GatingPolicy() = default;
  virtual bool passes(const Eigen::VectorXd &state, const Eigen::VectorXd &measurement) const = 0;
};

} // namespace rfs

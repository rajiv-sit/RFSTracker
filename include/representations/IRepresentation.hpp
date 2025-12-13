#pragma once

#include "simulation/MeasurementSet.hpp"

#include <Eigen/Dense>
#include <vector>

namespace rfs {

class IRepresentation {
public:
  virtual ~IRepresentation() = default;
  virtual void predict(double dt) = 0;
  virtual void update(const MeasurementSet_t &measurements) = 0;
  virtual std::vector<Eigen::Vector4d> estimate() const = 0;
};

} // namespace rfs

#pragma once

#include <Eigen/Dense>
#include <vector>

namespace rfs {

struct MeasurementSet_t;

/** @brief Common filter output summary. */
struct EstimatorOutput_t {
  int estimatedCount = 0;
  std::vector<Eigen::Vector4d> tracks;
  double nees = 0.0;
};

/** @brief Interface for RFSTracker filters. */
class IrfsFilter {
public:
  virtual ~IrfsFilter() = default;

  virtual void predict(double dt) = 0;
  virtual void update(const MeasurementSet_t &measurements) = 0;
  virtual EstimatorOutput_t estimate() const = 0;
};

} // namespace rfs

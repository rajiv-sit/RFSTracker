#pragma once

#include <Eigen/Dense>
#include <vector>

namespace rfs {

struct TrackEstimate_t {
  Eigen::Vector4d state = Eigen::Vector4d::Zero();
};

struct TruthTarget_t {
  Eigen::Vector4d state = Eigen::Vector4d::Zero();
};

/** @brief Computes filter performance metrics for tracking. */
class PerformanceEvaluator {
public:
  double computeNEES(const std::vector<TrackEstimate_t> &estimates,
                     const std::vector<TruthTarget_t> &truth) const;

  double computeRMSE(const std::vector<TrackEstimate_t> &estimates,
                     const std::vector<TruthTarget_t> &truth) const;

  double computeOSPA(const std::vector<TrackEstimate_t> &estimates,
                     const std::vector<TruthTarget_t> &truth) const;
};

} // namespace rfs

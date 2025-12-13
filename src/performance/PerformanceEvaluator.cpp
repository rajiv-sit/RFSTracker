#include "performance/PerformanceEvaluator.hpp"

#include <algorithm>
#include <cmath>

namespace rfs {

double PerformanceEvaluator::computeNEES(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  if (estimates.empty() || truth.empty()) {
    return 0.0;
  }

  const size_t count = std::min(estimates.size(), truth.size());
  double total = 0.0;
  for (size_t i = 0; i < count; ++i) {
    const Eigen::Vector4d error = estimates[i].state - truth[i].state;
    total += error.squaredNorm();
  }
  return total / static_cast<double>(count * 4);
}

double PerformanceEvaluator::computeRMSE(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  if (estimates.empty() || truth.empty()) {
    return 0.0;
  }

  const size_t count = std::min(estimates.size(), truth.size());
  double total = 0.0;
  for (size_t i = 0; i < count; ++i) {
    const Eigen::Vector4d error = estimates[i].state - truth[i].state;
    total += error.head<2>().squaredNorm();
  }
  return std::sqrt(total / static_cast<double>(count));
}

double PerformanceEvaluator::computeOSPA(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  if (estimates.empty() && truth.empty()) {
    return 0.0;
  }

  const size_t n = std::max(estimates.size(), truth.size());
  const size_t matched = std::min(estimates.size(), truth.size());
  double constLevel = 50.0;
  double sum = 0.0;

  for (size_t i = 0; i < matched; ++i) {
    const double distance = (estimates[i].state - truth[i].state).head<2>().norm();
    sum += std::min(constLevel, distance);
  }

  const double unmatched = constLevel * static_cast<double>(n - matched);
  const double ospa = (sum + unmatched) / static_cast<double>(n);
  return ospa;
}

} // namespace rfs

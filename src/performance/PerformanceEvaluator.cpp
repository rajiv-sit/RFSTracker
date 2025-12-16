#include "performance/PerformanceEvaluator.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>

namespace rfs {

double PerformanceEvaluator::computeNEES(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  if (estimates.empty() || truth.empty()) {
    return 0.0;
  }

  std::unordered_map<int, const TruthTarget_t *> truthById;
  truthById.reserve(truth.size());
  for (const auto &entry : truth) {
    if (entry.id >= 0) {
      truthById[entry.id] = &entry;
    }
  }

  double total = 0.0;
  size_t matched = 0;
  for (const auto &estimate : estimates) {
    if (!estimate.truthId) {
      continue;
    }
    const auto found = truthById.find(*estimate.truthId);
    if (found == truthById.end()) {
      continue;
    }
    const Eigen::Vector4d error = estimate.state - found->second->state;
    total += error.squaredNorm();
    ++matched;
  }

  if (matched == 0) {
    return 0.0;
  }
  return total / static_cast<double>(matched * 4);
}

double PerformanceEvaluator::computeRMSE(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  if (estimates.empty() || truth.empty()) {
    return 0.0;
  }

  std::unordered_map<int, const TruthTarget_t *> truthById;
  truthById.reserve(truth.size());
  for (const auto &entry : truth) {
    if (entry.id >= 0) {
      truthById[entry.id] = &entry;
    }
  }

  double total = 0.0;
  size_t matched = 0;
  for (const auto &estimate : estimates) {
    if (!estimate.truthId) {
      continue;
    }
    const auto found = truthById.find(*estimate.truthId);
    if (found == truthById.end()) {
      continue;
    }
    const Eigen::Vector4d error = estimate.state - found->second->state;
    total += error.head<2>().squaredNorm();
    ++matched;
  }

  if (matched == 0) {
    return 0.0;
  }
  return std::sqrt(total / static_cast<double>(matched));
}

double PerformanceEvaluator::computeOSPA(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  if (estimates.empty() && truth.empty()) {
    return 0.0;
  }

  std::unordered_map<int, const TruthTarget_t *> truthById;
  truthById.reserve(truth.size());
  for (const auto &entry : truth) {
    if (entry.id >= 0) {
      truthById[entry.id] = &entry;
    }
  }

  double constLevel = 50.0;
  double sum = 0.0;
  size_t matched = 0;
  for (const auto &estimate : estimates) {
    if (!estimate.truthId) {
      continue;
    }
    const auto found = truthById.find(*estimate.truthId);
    if (found == truthById.end()) {
      continue;
    }
    const double distance = (estimate.state - found->second->state).head<2>().norm();
    sum += std::min(constLevel, distance);
    ++matched;
  }

  const size_t totalEstimates = estimates.size();
  const size_t totalTruth = truth.size();
  const size_t n = std::max(totalEstimates, totalTruth);
  if (n == 0) {
    return 0.0;
  }

  const size_t unmatched = (n > matched) ? (n - matched) : 0;
  const double ospa = (sum + constLevel * static_cast<double>(unmatched)) /
                      static_cast<double>(n);
  return ospa;
}

} // namespace rfs

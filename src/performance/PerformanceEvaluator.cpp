#include "performance/PerformanceEvaluator.hpp"

#include <algorithm>
#include <cmath>
#include <unordered_map>
#include <utility>

namespace rfs {

namespace {
std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>>
matchEstimatesToTruth(const std::vector<TrackEstimate_t> &estimates,
                      const std::vector<TruthTarget_t> &truth) {
  std::unordered_map<int, const TruthTarget_t *> truthById;
  truthById.reserve(truth.size());
  for (const auto &entry : truth) {
    if (entry.id >= 0) {
      truthById[entry.id] = &entry;
    }
  }

  std::vector<std::pair<Eigen::Vector4d, Eigen::Vector4d>> pairs;
  for (const auto &estimate : estimates) {
    if (!estimate.truthId) {
      continue;
    }
    const auto found = truthById.find(*estimate.truthId);
    if (found == truthById.end()) {
      continue;
    }
    pairs.emplace_back(estimate.state, found->second->state);
  }

  if (pairs.empty() && !estimates.empty() && !truth.empty()) {
    const size_t count = std::min(estimates.size(), truth.size());
    for (size_t i = 0; i < count; ++i) {
      pairs.emplace_back(estimates[i].state, truth[i].state);
    }
  }
  return pairs;
}
} // namespace

double PerformanceEvaluator::computeNEES(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  const auto pairs = matchEstimatesToTruth(estimates, truth);
  if (pairs.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &pair : pairs) {
    const Eigen::Vector4d error = pair.first - pair.second;
    total += error.squaredNorm();
  }
  return total / static_cast<double>(pairs.size() * 4);
}

double PerformanceEvaluator::computeRMSE(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  const auto pairs = matchEstimatesToTruth(estimates, truth);
  if (pairs.empty()) {
    return 0.0;
  }

  double total = 0.0;
  for (const auto &pair : pairs) {
    const Eigen::Vector4d error = pair.first - pair.second;
    total += error.head<2>().squaredNorm();
  }
  return std::sqrt(total / static_cast<double>(pairs.size()));
}

double PerformanceEvaluator::computeOSPA(
    const std::vector<TrackEstimate_t> &estimates,
    const std::vector<TruthTarget_t> &truth) const {
  if (estimates.empty() && truth.empty()) {
    return 0.0;
  }

  const auto pairs = matchEstimatesToTruth(estimates, truth);
  double constLevel = 50.0;
  double sum = 0.0;
  for (const auto &pair : pairs) {
    const double distance = (pair.first - pair.second).head<2>().norm();
    sum += std::min(constLevel, distance);
  }

  const size_t totalEstimates = estimates.size();
  const size_t totalTruth = truth.size();
  const size_t n = std::max(totalEstimates, totalTruth);
  if (n == 0) {
    return 0.0;
  }
  const size_t matched = pairs.size();
  const size_t unmatched = (n > matched) ? (n - matched) : 0;
  const double ospa = (sum + constLevel * static_cast<double>(unmatched)) /
                      static_cast<double>(n);
  return ospa;
}

} // namespace rfs

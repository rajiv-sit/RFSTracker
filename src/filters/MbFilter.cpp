#include "filters/MbFilter.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rfs {

namespace {
constexpr double kGatingThreshold = 30.0;
constexpr double kExistenceDecay = 0.03;
constexpr double kExistenceBoost = 0.18;
}

MbFilter::MbFilter(RepresentationType representationType, const TrackerConfig *config)
    : representationType_(representationType), config_(config) {}

void MbFilter::predict(double dt) {
  for (auto &hypothesis : hypotheses_) {
    hypothesis.representation->predict(dt);
    hypothesis.existence = std::max(0.0, hypothesis.existence - 0.005);
  }
}

void MbFilter::update(const MeasurementSet_t &measurements) {
  if (measurements.measurements.empty()) {
    for (auto &hypothesis : hypotheses_) {
      hypothesis.existence = std::max(0.0, hypothesis.existence - kExistenceDecay);
    }
    pruneHypotheses();
    return;
  }

  std::vector<bool> assigned(measurements.measurements.size(), false);
  for (auto &hypothesis : hypotheses_) {
    const auto states = hypothesis.representation->estimate();
    bool hit = false;
    size_t bestIdx = 0;
    double bestDist = std::numeric_limits<double>::infinity();
    if (!states.empty()) {
      const auto position = states.front().head<2>();
      for (size_t measurementIdx = 0; measurementIdx < measurements.measurements.size();
           ++measurementIdx) {
        const double distance =
            (position - measurements.measurements[measurementIdx].value).norm();
        if (distance < bestDist) {
          bestDist = distance;
          bestIdx = measurementIdx;
        }
      }
      if (bestDist < kGatingThreshold) {
        hit = true;
        assigned[bestIdx] = true;
      }
    }

    MeasurementSet_t localMeasurements;
    if (hit) {
      localMeasurements.measurements.push_back(measurements.measurements[bestIdx]);
      hypothesis.representation->update(localMeasurements);
      hypothesis.existence =
          std::min(1.0, hypothesis.existence + kExistenceBoost * detectionProbability());
    } else {
      hypothesis.existence = std::max(0.0, hypothesis.existence - kExistenceDecay);
    }
  }

  for (size_t idx = 0; idx < measurements.measurements.size(); ++idx) {
    if (assigned[idx]) {
      continue;
    }
    BernoulliHypothesis born;
    born.representation = createEmptyRepresentation();
    MeasurementSet_t localMeasurements;
    localMeasurements.measurements.push_back(measurements.measurements[idx]);
    born.representation->update(localMeasurements);
    born.existence = 0.5;
    hypotheses_.push_back(std::move(born));
  }

  pruneHypotheses();
}

EstimatorOutput_t MbFilter::estimate() const {
  EstimatorOutput_t output;
  double totalExistence = 0.0;
  for (const auto &hypothesis : hypotheses_) {
    const auto states = hypothesis.representation->estimate();
    if (!states.empty()) {
      output.tracks.push_back(states.front());
    }
    totalExistence += hypothesis.existence;
  }
  output.estimatedCount = static_cast<int>(std::round(totalExistence));
  return output;
}

const std::vector<MbFilter::BernoulliHypothesis> &MbFilter::hypotheses() const {
  return hypotheses_;
}

std::unique_ptr<IRepresentation> MbFilter::createEmptyRepresentation() const {
  return createRepresentation(representationType_);
}

void MbFilter::pruneHypotheses() {
  hypotheses_.erase(
      std::remove_if(hypotheses_.begin(), hypotheses_.end(),
                     [](const BernoulliHypothesis &hypothesis) {
                       return hypothesis.existence < 0.05;
                     }),
      hypotheses_.end());
  if (config_ && static_cast<int>(hypotheses_.size()) > config_->maxTargets) {
    hypotheses_.resize(config_->maxTargets);
  }
}

double MbFilter::detectionProbability() const {
  if (!config_ || config_->sensors.empty()) {
    return 0.9;
  }
  return config_->sensors.front().detectionProbability;
}

} // namespace rfs

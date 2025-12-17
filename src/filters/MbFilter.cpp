#include "filters/FilterLogger.hpp"
#include "filters/MbFilter.hpp"

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

namespace rfs {

namespace {
constexpr double kGatingThreshold = 30.0;
constexpr double kExistenceDecay = 0.03;
constexpr double kExistenceBoost = 0.18;
std::string formatMeasurementDetail(const Measurement_t &measurement) {
  std::ostringstream oss;
  oss << "t=" << measurement.time << " pos=(" << measurement.value.x() << "," << measurement.value.y()
      << ")";
  oss << " truth=";
  if (measurement.truthId) {
    oss << *measurement.truthId;
  } else {
    oss << "unknown";
  }
  return oss.str();
}
}

MbFilter::MbFilter(RepresentationType representationType, const TrackerConfig *config)
    : representationType_(representationType), config_(config) {}

void MbFilter::predict(double dt) {
  std::ostringstream detail;
  detail << "dt=" << dt << " hypothesisCount=" << hypotheses_.size();
  logFilterAction("MbFilter", "predict", detail.str());
  for (auto &hypothesis : hypotheses_) {
    hypothesis.representation->predict(dt);
    hypothesis.existence = std::max(0.0, hypothesis.existence - 0.005);
  }
}

void MbFilter::update(const MeasurementSet_t &measurements) {
  std::ostringstream detail;
  detail << "measurements=" << measurements.measurements.size();
  logFilterAction("MbFilter", "update", detail.str());
  if (measurements.measurements.empty()) {
    for (auto &hypothesis : hypotheses_) {
      hypothesis.existence = std::max(0.0, hypothesis.existence - kExistenceDecay);
    }
    pruneHypotheses();
    return;
  }

  const size_t hypCount = hypotheses_.size();
  const size_t measurementCount = measurements.measurements.size();
  std::vector<std::vector<double>> costMatrix(hypCount, std::vector<double>(measurementCount));
  std::vector<std::string> measurementLabels(measurementCount);
  for (size_t j = 0; j < measurementCount; ++j) {
    measurementLabels[j] = formatMeasurementDetail(measurements.measurements[j]);
  }
  for (size_t i = 0; i < hypCount; ++i) {
    const auto states = hypotheses_[i].representation->estimate();
    Eigen::Vector2d statePos = Eigen::Vector2d::Zero();
    if (!states.empty()) {
      statePos = states.front().head<2>();
    }
    for (size_t j = 0; j < measurementCount; ++j) {
      costMatrix[i][j] = (statePos - measurements.measurements[j].value).norm();
    }
  }

  AssociationResult_t assignment = solver_.solve(costMatrix);
  std::vector<bool> measurementAssigned(measurementCount, false);
  std::vector<int> assignedMeasurementIndices(hypCount, -1);

  for (size_t i = 0; i < hypCount; ++i) {
    auto &hypothesis = hypotheses_[i];
    int measurementIdx = -1;
    if (i < assignment.assignment.size()) {
      measurementIdx = assignment.assignment[i];
    }
    if (measurementIdx >= 0 && static_cast<size_t>(measurementIdx) < measurementCount) {
      const double distance = costMatrix[i][measurementIdx];
      if (distance > kGatingThreshold) {
        measurementIdx = -1;
      }
    }
    if (measurementIdx >= 0 && static_cast<size_t>(measurementIdx) < measurementCount) {
      MeasurementSet_t localMeasurements;
      localMeasurements.measurements.push_back(measurements.measurements[measurementIdx]);
      hypothesis.representation->update(localMeasurements);
      hypothesis.existence =
          std::min(1.0, hypothesis.existence + kExistenceBoost * detectionProbability());
      measurementAssigned[measurementIdx] = true;
      assignedMeasurementIndices[i] = measurementIdx;
    } else {
      hypothesis.existence = std::max(0.0, hypothesis.existence - kExistenceDecay);
    }
  }

  for (size_t idx = 0; idx < measurementCount; ++idx) {
    if (measurementAssigned[idx]) {
      continue;
    }
    BernoulliHypothesis born;
    born.representation = createEmptyRepresentation();
    MeasurementSet_t localMeasurements;
    localMeasurements.measurements.push_back(measurements.measurements[idx]);
    born.representation->update(localMeasurements);
    born.existence = 0.5;
    born.id = nextHypothesisId_++;
    hypotheses_.push_back(std::move(born));
    assignedMeasurementIndices.push_back(static_cast<int>(idx));
  }

  std::ostringstream updateDetail;
  updateDetail << "postUpdate hypCount=" << hypotheses_.size();
  for (size_t i = 0; i < hypotheses_.size(); ++i) {
    const auto &hypothesis = hypotheses_[i];
    updateDetail << " [hyp" << hypothesis.id << " exist=" << std::fixed << std::setprecision(2)
                 << hypothesis.existence;
    int assignedIdx = (i < assignedMeasurementIndices.size()) ? assignedMeasurementIndices[i] : -1;
    if (assignedIdx >= 0) {
      updateDetail << " assigned=" << measurementLabels[assignedIdx];
    } else {
      updateDetail << " assigned=none";
    }
    updateDetail << "]";
  }
  logFilterAction("MbFilter", "updateDetail", updateDetail.str());

  pruneHypotheses();
  std::ostringstream pruneDetail;
  pruneDetail << "postPrune hypCount=" << hypotheses_.size();
  logFilterAction("MbFilter", "prune", pruneDetail.str());
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
  return createRepresentation(representationType_, config_);
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

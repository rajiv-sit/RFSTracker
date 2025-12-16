#include "association/AssociationLogger.hpp"
#include "filters/CphdFilter.hpp"
#include "filters/FilterLogger.hpp"
#include "filters/GlmbFilter.hpp"
#include "filters/MbFilter.hpp"
#include "filters/PhdFilter.hpp"
#include "logging/LoggerControl.hpp"
#include "logging/TruthTrackLogger.hpp"
#include "pipeline/TrackingPipeline.hpp"
#include "representations/RepresentationFactory.hpp"
#include "representations/RepresentationLogger.hpp"
#include "simulation/TargetState.hpp"
#include "track/TrackLogger.hpp"

#include <Eigen/Dense>
#include <fstream>
#include <limits>
#include <optional>
#include <unordered_map>
#include <utility>

namespace rfs {

namespace {
void logPipeline(const std::string &message) {
  if (!loggerVerboseEnabled()) {
    return;
  }
  std::ofstream log("rfs_app_debug.log", std::ios::app);
  log << message << std::endl;
}

std::optional<size_t> matchTruthTarget(const Eigen::Vector2d &position,
                                       const std::vector<TargetState_t> &truthTargets,
                                       std::vector<bool> &assigned, double threshold) {
  std::optional<size_t> bestIndex;
  double bestDistance = std::numeric_limits<double>::max();

  for (size_t i = 0; i < truthTargets.size(); ++i) {
    if (assigned[i]) {
      continue;
    }
    const double distance = (truthTargets[i].state.head<2>() - position).norm();
    if (distance < bestDistance) {
      bestDistance = distance;
      bestIndex = i;
    }
  }

  if (bestIndex && bestDistance <= threshold) {
    assigned[*bestIndex] = true;
    return bestIndex;
  }
  return std::nullopt;
}

double computeTrackSpread(const std::vector<TrackState> &tracks) {
  const size_t count = tracks.size();
  if (count < 2) {
    return 0.0;
  }

  double totalDistance = 0.0;
  size_t pairCount = 0;
  for (size_t i = 0; i < count; ++i) {
    for (size_t j = i + 1; j < count; ++j) {
      totalDistance += (tracks[i].position - tracks[j].position).norm();
      ++pairCount;
    }
  }

  if (pairCount == 0) {
    return 0.0;
  }
  return totalDistance / static_cast<double>(pairCount);
}
} // namespace

TrackingPipeline::TrackingPipeline(std::shared_ptr<TrackerConfig> config,
                                   std::unique_ptr<Visualizer> visualizer)
    : config_(std::move(config)),
      simulation_(*config_),
      visualizer_(std::move(visualizer)) {
  instantiateFilter();
}

TrackingPipeline::~TrackingPipeline() {
  if (visualizer_) {
    visualizer_->shutdown();
  }
}

void TrackingPipeline::step(double dt) {
  logPipeline("pipeline: step start");
  setRepresentationScanId(scanId_);
  simulation_.step(currentTime_, dt, measurementSet_);
  logPipeline("pipeline: simulation step complete");
  if (filter_) {
    setFilterScanId(scanId_);
    filter_->predict(dt);
    logPipeline("pipeline: filter predict complete");
    setFilterScanId(scanId_);
    filter_->update(measurementSet_);
    logPipeline("pipeline: filter update complete");
  }

  logPipeline("pipeline: track update start");
  setTrackScanId(scanId_);
  setAssociationScanId(scanId_);
  const auto &truthTargets = simulation_.truthStates();
  std::vector<bool> truthAssigned(truthTargets.size(), false);
  MeasurementSet_t estimateMeasurements;
  if (filter_) {
    const auto estimatorOutput = filter_->estimate();
    estimateMeasurements.measurements.reserve(estimatorOutput.tracks.size());
    const double truthMatchThreshold = (config_ && config_->truthMatchThreshold > 0.0)
                                           ? config_->truthMatchThreshold
                                           : std::numeric_limits<double>::max();
    for (const auto &state : estimatorOutput.tracks) {
      Measurement_t measurement;
      measurement.value = state.head<2>();
      measurement.time = currentTime_;
      if (const auto matched =
              matchTruthTarget(measurement.value, truthTargets, truthAssigned, truthMatchThreshold);
          matched) {
        measurement.truthId = truthTargets[*matched].id;
      }
      estimateMeasurements.measurements.push_back(measurement);
    }
  }
  trackManager_.update(estimateMeasurements);
  logPipeline("pipeline: track update complete");

  std::unordered_map<int, TargetState_t> truthById;
  truthById.reserve(truthTargets.size());
  for (const auto &target : truthTargets) {
    truthById[target.id] = target;
  }
  for (const auto &track : trackManager_.tracks()) {
    if (!track.truthId) {
      continue;
    }
    const auto found = truthById.find(*track.truthId);
    if (found == truthById.end()) {
      continue;
    }
    const double distance =
        (track.position - found->second.state.head<2>()).norm();
    logTruthTrackComparison(scanId_, track, found->second, distance);
  }

  const auto estimates = buildEstimates();
  const auto truth = buildTruth();
  logPipeline("pipeline: estimates/truth built");

  PerformanceMetrics metrics;
  metrics.truthAvailable = !truth.empty();
  if (metrics.truthAvailable) {
    metrics.nees = performance_.computeNEES(estimates, truth);
    metrics.rmse = performance_.computeRMSE(estimates, truth);
    metrics.ospa = performance_.computeOSPA(estimates, truth);
  } else {
    metrics.trackSpread = computeTrackSpread(trackManager_.confirmedTracks());
  }
  logPipeline("pipeline: metrics computed");

  if (visualizer_) {
    visualizer_->renderFrame(measurementSet_, trackManager_.confirmedTracks(), simulation_.truthStates(),
                             metrics, scanId_ + 1, currentTime_);
    logPipeline("pipeline: render frame");
  }

  currentTime_ += dt;
  ++scanId_;
  logPipeline("pipeline: step complete");
}

bool TrackingPipeline::shouldStop() const {
  return visualizer_ && visualizer_->shouldClose();
}

void TrackingPipeline::instantiateFilter() {
  switch (config_->filterFamily) {
  case FilterFamily::CPHD: {
    auto representation = createRepresentation(config_->representation, config_.get());
    filter_ = std::make_unique<CphdFilter>(std::move(representation));
    break;
  }
  case FilterFamily::MB:
    filter_ = std::make_unique<MbFilter>(config_->representation, config_.get());
    break;
  case FilterFamily::GLMB:
    filter_ = std::make_unique<GlmbFilter>(config_->representation, config_.get());
    break;
  case FilterFamily::PHD:
  default: {
    auto representation = createRepresentation(config_->representation, config_.get());
    filter_ = std::make_unique<PhdFilter>(std::move(representation));
    break;
  }
  }
}

std::vector<TrackEstimate_t> TrackingPipeline::buildEstimates() const {
  std::vector<TrackEstimate_t> result;
  const auto &confirmedTracks = trackManager_.confirmedTracks();
  result.reserve(confirmedTracks.size());

  for (const auto &track : confirmedTracks) {
    TrackEstimate_t estimate;
    estimate.state << track.position.x(), track.position.y(), track.velocity.x(),
        track.velocity.y();
    estimate.truthId = track.truthId;
    result.push_back(estimate);
  }

  return result;
}

std::vector<TruthTarget_t> TrackingPipeline::buildTruth() const {
  std::vector<TruthTarget_t> result;
  result.reserve(simulation_.truthStates().size());

  for (const auto &target : simulation_.truthStates()) {
    TruthTarget_t truth;
    truth.id = target.id;
    truth.state = target.state;
    result.push_back(truth);
  }

  return result;
}

} // namespace rfs

#include "filters/CphdFilter.hpp"
#include "filters/GlmbFilter.hpp"
#include "filters/MbFilter.hpp"
#include "filters/PhdFilter.hpp"
#include "pipeline/TrackingPipeline.hpp"
#include "representations/RepresentationFactory.hpp"

#include <fstream>
#include <utility>

namespace rfs {

namespace {
void logPipeline(const std::string &message) {
  std::ofstream log("rfs_app_debug.log", std::ios::app);
  log << message << std::endl;
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
  simulation_.step(currentTime_, dt, measurementSet_);
  logPipeline("pipeline: simulation step complete");
  if (filter_) {
    filter_->predict(dt);
    logPipeline("pipeline: filter predict complete");
    filter_->update(measurementSet_);
    logPipeline("pipeline: filter update complete");
  }

  logPipeline("pipeline: track update start");
  trackManager_.update(measurementSet_);
  logPipeline("pipeline: track update complete");

  const auto estimates = buildEstimates();
  const auto truth = buildTruth();
  logPipeline("pipeline: estimates/truth built");

  const double nees = performance_.computeNEES(estimates, truth);
  const double rmse = performance_.computeRMSE(estimates, truth);
  const double ospa = performance_.computeOSPA(estimates, truth);
  logPipeline("pipeline: metrics computed");

  if (visualizer_) {
    PerformanceMetrics metrics{rmse, nees, ospa};
    visualizer_->renderFrame(measurementSet_, trackManager_.tracks(), simulation_.truthStates(),
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
    auto representation = createRepresentation(config_->representation);
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
    auto representation = createRepresentation(config_->representation);
    filter_ = std::make_unique<PhdFilter>(std::move(representation));
    break;
  }
  }
}

std::vector<TrackEstimate_t> TrackingPipeline::buildEstimates() const {
  std::vector<TrackEstimate_t> result;
  result.reserve(trackManager_.tracks().size());

  for (const auto &track : trackManager_.tracks()) {
    TrackEstimate_t estimate;
    estimate.state << track.position.x(), track.position.y(), 0.0, 0.0;
    result.push_back(estimate);
  }

  return result;
}

std::vector<TruthTarget_t> TrackingPipeline::buildTruth() const {
  std::vector<TruthTarget_t> result;
  result.reserve(simulation_.truthStates().size());

  for (const auto &target : simulation_.truthStates()) {
    TruthTarget_t truth;
    truth.state = target.state;
    result.push_back(truth);
  }

  return result;
}

} // namespace rfs

#pragma once

#include "config/TrackerConfig.hpp"
#include "filters/CphdFilter.hpp"
#include "filters/GlmbFilter.hpp"
#include "filters/IrfsFilter.hpp"
#include "filters/MbFilter.hpp"
#include "filters/PhdFilter.hpp"
#include "performance/PerformanceEvaluator.hpp"
#include "simulation/MeasurementSet.hpp"
#include "simulation/SimulationEnvironment.hpp"
#include "track/TrackManager.hpp"
#include "visualization/Visualizer.hpp"
#include "representations/RepresentationFactory.hpp"

#include <memory>

namespace rfs {

/** @brief Orchestrates simulation, filtering, and visualization updates. */
class TrackingPipeline {
public:
  TrackingPipeline(std::shared_ptr<TrackerConfig> config,
                   std::unique_ptr<Visualizer> visualizer);
  ~TrackingPipeline();

  /** @brief Run a single pipeline step. */
  void step(double dt);

  bool shouldStop() const;

private:
  void instantiateFilter();
  std::vector<TrackEstimate_t> buildEstimates() const;
  std::vector<TruthTarget_t> buildTruth() const;

  std::shared_ptr<TrackerConfig> config_;
  std::unique_ptr<IrfsFilter> filter_;
  SimulationEnvironment simulation_;
  TrackManager trackManager_;
  PerformanceEvaluator performance_;
  std::unique_ptr<Visualizer> visualizer_;
  MeasurementSet_t measurementSet_;
  double currentTime_ = 0.0;
  int scanId_ = 0;
};

} // namespace rfs

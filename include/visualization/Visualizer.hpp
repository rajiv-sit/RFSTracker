#pragma once

#include "simulation/MeasurementSet.hpp"
#include "simulation/TargetState.hpp"
#include "track/TrackManager.hpp"

#include <vector>

namespace rfs {

struct PerformanceMetrics {
  double rmse = 0.0;
  double nees = 0.0;
  double ospa = 0.0;
  double trackSpread = 0.0;
  bool truthAvailable = true;
  // Future metrics (NIS, OSPA2, etc.) can be added here.
};

struct VisualizerOptions {
  bool showTruth = true;
  bool showMeasurements = true;
  bool showTracks = true;
  bool showTrackDetails = true;
  bool showTruthDetails = true;
  bool showTrackTruthComparison = false;
};

/** @brief Runtime polymorphic visualizer contract. */
class Visualizer {
public:
  virtual ~Visualizer() = default;

  /** @brief Prepare rendering resources (window, context). */
  virtual bool initialize() = 0;

  /** @brief Render a single frame using the latest snapshot. */
  virtual void renderFrame(const MeasurementSet_t &measurements,
                           const std::vector<TrackState> &tracks,
                           const std::vector<TargetState_t> &truth,
                           const PerformanceMetrics &metrics,
                           int scanId,
                           double timeElapsed) = 0;

  /** @brief Tear down rendering resources. */
  virtual void shutdown() = 0;

  virtual const VisualizerOptions &options() const = 0;
  virtual void setOptions(const VisualizerOptions &options) = 0;
  virtual bool shouldClose() const = 0;
};

} // namespace rfs

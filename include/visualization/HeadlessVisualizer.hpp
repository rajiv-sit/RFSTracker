#pragma once

#include "visualization/Visualizer.hpp"

#include <memory>

namespace rfs {

class HeadlessVisualizer final : public Visualizer {
public:
  explicit HeadlessVisualizer(std::shared_ptr<TrackerConfig> config)
      : config_(std::move(config)) {}

  bool initialize() override { return true; }

  void renderFrame(const MeasurementSet_t &, const std::vector<TrackState> &,
                   const std::vector<TargetState_t> &, const PerformanceMetrics &, int,
                   double) override {}

  void shutdown() override {}

  const VisualizerOptions &options() const override { return options_; }

  void setOptions(const VisualizerOptions &options) override { options_ = options; }

  bool shouldClose() const override { return shouldClose_; }

private:
  std::shared_ptr<TrackerConfig> config_;
  VisualizerOptions options_;
  bool shouldClose_ = false;
};

} // namespace rfs

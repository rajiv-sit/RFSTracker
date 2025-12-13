#pragma once

#include "config/TrackerConfig.hpp"
#include "simulation/TargetState.hpp"

#include <vector>

namespace rfs {

/** @brief Advances truth targets through motion models. */
class GroundTruthGenerator {
public:
  explicit GroundTruthGenerator(const TrackerConfig &config);

  void step(double currentTime, double dt);

  const std::vector<TargetState_t> &activeTargets() const;

private:
  void updateState(TargetState_t &state, const TargetDescriptor &descriptor,
                   double currentTime, double dt);

  const TrackerConfig &config_;
  std::vector<TargetState_t> states_;
  std::vector<TargetState_t> activeTargets_;
};

} // namespace rfs

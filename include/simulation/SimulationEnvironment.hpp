#pragma once

#include "config/TrackerConfig.hpp"
#include "simulation/GroundTruthGenerator.hpp"
#include "simulation/MeasurementSet.hpp"
#include "simulation/TargetState.hpp"
#include "simulation/SensorInterface.hpp"

#include <memory>
#include <vector>

namespace rfs {

/** @brief Encapsulates the truth generator and sensors for a simulation step. */
class SimulationEnvironment {
public:
  explicit SimulationEnvironment(const TrackerConfig &config);

  void step(double currentTime, double dt, MeasurementSet_t &measurements);

  const std::vector<TargetState_t> &truthStates() const;

private:
  const TrackerConfig &config_;
  GroundTruthGenerator truthGenerator_;
  std::vector<std::unique_ptr<SensorInterface>> sensors_;
};

} // namespace rfs

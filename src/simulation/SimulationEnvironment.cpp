#include "simulation/SimulationEnvironment.hpp"

#include <memory>

namespace rfs {

SimulationEnvironment::SimulationEnvironment(const TrackerConfig &config)
    : config_(config), truthGenerator_(config_) {
  for (size_t idx = 0; idx < config_.sensors.size(); ++idx) {
    const auto &sensorConfig = config_.sensors[idx];
    if (sensorConfig.type == SensorType::Radar) {
      sensors_.push_back(std::make_unique<RadarSensor>(static_cast<int>(idx), sensorConfig));
    } else {
      sensors_.push_back(std::make_unique<RadarSensor>(static_cast<int>(idx), sensorConfig));
    }
  }
}

void SimulationEnvironment::step(double currentTime, double dt, MeasurementSet_t &measurements) {
  truthGenerator_.step(currentTime, dt);
  measurements.clear();

  for (const auto &sensor : sensors_) {
    sensor->sense(currentTime, truthGenerator_.activeTargets(), measurements);
  }
}

const std::vector<TargetState_t> &SimulationEnvironment::truthStates() const {
  return truthGenerator_.activeTargets();
}

} // namespace rfs

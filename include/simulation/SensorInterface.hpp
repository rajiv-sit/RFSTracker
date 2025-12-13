#pragma once

#include "config/TrackerConfig.hpp"
#include "simulation/MeasurementSet.hpp"
#include "simulation/TargetState.hpp"

#include <random>

namespace rfs {

/** @brief Base sensor handling detection, FoV, and noise. */
class SensorInterface {
public:
  SensorInterface(int sensorId, SensorConfig config);
  virtual ~SensorInterface() = default;

  virtual void sense(double time, const std::vector<TargetState_t> &truth,
                     MeasurementSet_t &out) const = 0;

  int id() const { return sensorId_; }

protected:
  int sensorId_;
  SensorConfig config_;
  mutable std::mt19937 rng_;
};

class RadarSensor : public SensorInterface {
public:
  explicit RadarSensor(int sensorId, const SensorConfig &config);

  void sense(double time, const std::vector<TargetState_t> &truth,
             MeasurementSet_t &out) const override;

private:
  void addClutter(double time, MeasurementSet_t &out) const;
};

} // namespace rfs

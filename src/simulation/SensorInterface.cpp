#include "simulation/SensorInterface.hpp"

#include <algorithm>
#include <cmath>
#include <random>
#include <utility>

namespace rfs {

SensorInterface::SensorInterface(int sensorId, SensorConfig config)
    : sensorId_(sensorId), config_(std::move(config)), rng_(std::random_device{}()) {}

RadarSensor::RadarSensor(int sensorId, const SensorConfig &config)
    : SensorInterface(sensorId, config) {}

void RadarSensor::sense(double time, const std::vector<TargetState_t> &truth,
                        MeasurementSet_t &out) const {
  std::bernoulli_distribution detection(config_.detectionProbability);
  std::normal_distribution<double> noiseX(0.0, config_.rangeStdDev);
  std::normal_distribution<double> noiseY(0.0, config_.rangeStdDev);

  for (const auto &track : truth) {
    if (!detection(rng_)) {
      continue;
    }

    Measurement_t measurement;
    measurement.sensorId = sensorId_;
    measurement.time = time;
    measurement.truthId = track.id;
    measurement.isClutter = false;
    measurement.snrDb = config_.signalToNoiseDb;
    measurement.value.x() = track.state.x() + noiseX(rng_);
    measurement.value.y() = track.state.y() + noiseY(rng_);
    out.measurements.push_back(measurement);
  }

  addClutter(time, out);
}

void RadarSensor::addClutter(double time, MeasurementSet_t &out) const {
  if (config_.falseAlarmRate <= 0.0) {
    return;
  }
  std::poisson_distribution<int> clutterCount(config_.falseAlarmRate);
  std::uniform_real_distribution<double> distX(-config_.range, config_.range);
  std::uniform_real_distribution<double> distY(-config_.range, config_.range);

  const int count = std::max(0, clutterCount(rng_));
  for (int idx = 0; idx < count; ++idx) {
    Measurement_t clutter;
    clutter.sensorId = sensorId_;
    clutter.time = time;
    clutter.truthId.reset();
    clutter.isClutter = true;
    clutter.snrDb = config_.signalToNoiseDb - 10.0;
    clutter.value.x() = distX(rng_);
    clutter.value.y() = distY(rng_);
    out.measurements.push_back(clutter);
  }
}

} // namespace rfs

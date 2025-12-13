#pragma once

#pragma once

#include <optional>
#include <vector>

#include <Eigen/Dense>

namespace rfs {

/** @brief 2D measurement emitted by sensors. */
struct Measurement_t {
  int sensorId = -1;
  double time = 0.0;
  Eigen::Vector2d value = Eigen::Vector2d::Zero();
  std::optional<int> truthId;
  bool isClutter = false;
  double snrDb = 0.0;
};

/** @brief Collection of measurements across sensors per scan. */
struct MeasurementSet_t {
  std::vector<Measurement_t> measurements;

  void clear() { measurements.clear(); }
};

} // namespace rfs

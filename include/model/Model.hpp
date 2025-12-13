#pragma once

#include <memory>
#include <vector>

namespace rfs {

class MotionModel;
class MeasurementModel;

/** @brief Aggregates motion, measurement, and clutter/birth models. */
class Model {
public:
  explicit Model(std::shared_ptr<MotionModel> motion,
                 std::shared_ptr<MeasurementModel> measurement);

  /** @brief Predicts the state forward by dt. */
  void predictState(double dt);

  /** @brief Evaluate likelihood for a measurement. */
  double evaluateLikelihood() const;

private:
  std::shared_ptr<MotionModel> motionModel_;
  std::shared_ptr<MeasurementModel> measurementModel_;
};

} // namespace rfs

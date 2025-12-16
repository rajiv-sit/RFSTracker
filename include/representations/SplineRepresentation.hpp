#pragma once

#include "representations/IRepresentation.hpp"

#include <Eigen/Dense>
#include <bspline.h>
#include <map>
#include <memory>
#include <vector>
#include <optional>

namespace SPLINTER {
class BSpline;
class DataTable;
}

namespace rfs {

/** @brief Spline-based state interpolation backend. */
class SplineRepresentation : public IRepresentation {
public:
  void predict(double /*dt*/) override {}
  void update(const MeasurementSet_t &measurements) override;
  std::vector<Eigen::Vector4d> estimate() const override;

private:
  std::unique_ptr<SPLINTER::BSpline> buildSpline(const SPLINTER::DataTable &table) const;

  struct TrackHistory {
    std::map<double, Eigen::Vector2d> samples;
    double latestTime = 0.0;
    std::unique_ptr<SPLINTER::BSpline> bsplineX;
    std::unique_ptr<SPLINTER::BSpline> bsplineY;
  };
  std::map<std::optional<int>, TrackHistory> trackHistories_;
  static constexpr size_t numBasisFunctions_ = 12;
  static constexpr size_t minSplineSamples_ = 5;
  static constexpr size_t maxHistorySamples_ = 128;
};

} // namespace rfs

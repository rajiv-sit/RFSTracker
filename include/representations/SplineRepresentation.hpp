#pragma once

#include "representations/IRepresentation.hpp"

#include <Eigen/Dense>
#include <bspline.h>
#include <memory>
#include <vector>

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

  std::unique_ptr<SPLINTER::BSpline> bsplineX_;
  std::unique_ptr<SPLINTER::BSpline> bsplineY_;
  std::vector<double> evaluationTimes_;
};

} // namespace rfs

#include "representations/SplineRepresentation.hpp"

#include <bspline.h>
#include <bsplinebuilder.h>
#include <datatable.h>

#include <algorithm>

namespace rfs {

void SplineRepresentation::update(const MeasurementSet_t &measurements) {
  evaluationTimes_.clear();
  if (measurements.measurements.empty()) {
    bsplineX_.reset();
    bsplineY_.reset();
    return;
  }

  SPLINTER::DataTable dataX(true);
  SPLINTER::DataTable dataY(true);

  for (const auto &measurement : measurements.measurements) {
    dataX.addSample(measurement.time, measurement.value.x());
    dataY.addSample(measurement.time, measurement.value.y());
    evaluationTimes_.push_back(measurement.time);
  }

  bsplineX_ = buildSpline(dataX);
  bsplineY_ = buildSpline(dataY);

  std::sort(evaluationTimes_.begin(), evaluationTimes_.end());
  evaluationTimes_.erase(std::unique(evaluationTimes_.begin(), evaluationTimes_.end()),
                         evaluationTimes_.end());
}

std::vector<Eigen::Vector4d> SplineRepresentation::estimate() const {
  std::vector<Eigen::Vector4d> states;
  if (!bsplineX_ || !bsplineY_ || evaluationTimes_.empty()) {
    return states;
  }

  states.reserve(evaluationTimes_.size());
  for (double time : evaluationTimes_) {
    SPLINTER::DenseVector input(1);
    input[0] = time;
    Eigen::Vector4d state;
    state[0] = bsplineX_->eval(input);
    state[1] = bsplineY_->eval(input);
    state[2] = 0.0;
    state[3] = 0.0;
    states.push_back(state);
  }

  return states;
}

std::unique_ptr<SPLINTER::BSpline> SplineRepresentation::buildSpline(
    const SPLINTER::DataTable &table) const {
  if (table.getNumSamples() < 4) {
    return nullptr;
  }

  SPLINTER::BSpline::Builder builder(table);
  builder.degree(3)
      .numBasisFunctions(12)
      .knotSpacing(SPLINTER::BSpline::KnotSpacing::EQUIDISTANT)
      .smoothing(SPLINTER::BSpline::Smoothing::NONE);

  return std::make_unique<SPLINTER::BSpline>(builder.build());
}

} // namespace rfs

#include "representations/SplineRepresentation.hpp"

#include <bspline.h>
#include <bsplinebuilder.h>
#include <datatable.h>

#include <algorithm>

namespace rfs {

void SplineRepresentation::update(const MeasurementSet_t &measurements) {
  if (measurements.measurements.empty()) {
    return;
  }

  const double time = measurements.measurements.front().time;
  Eigen::Vector2d sum = Eigen::Vector2d::Zero();
  for (const auto &measurement : measurements.measurements) {
    sum += measurement.value;
  }
  measurementHistory_[time] = sum / static_cast<double>(measurements.measurements.size());
  if (measurementHistory_.size() > maxHistorySamples_) {
    measurementHistory_.erase(measurementHistory_.begin());
  }

  if (measurementHistory_.size() < minSplineSamples_) {
    bsplineX_.reset();
    bsplineY_.reset();
    evaluationTimes_.clear();
    return;
  }

  SPLINTER::DataTable dataX(true);
  SPLINTER::DataTable dataY(true);
  evaluationTimes_.clear();

  for (const auto &entry : measurementHistory_) {
    dataX.addSample(entry.first, entry.second.x());
    dataY.addSample(entry.first, entry.second.y());
    evaluationTimes_.push_back(entry.first);
  }

  bsplineX_ = buildSpline(dataX);
  bsplineY_ = buildSpline(dataY);
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
    try {
      Eigen::Vector4d state;
      state[0] = bsplineX_->eval(input);
      state[1] = bsplineY_->eval(input);
      state[2] = 0.0;
      state[3] = 0.0;
      states.push_back(state);
    } catch (const std::exception &) {
      continue;
    }
  }

  return states;
}

std::unique_ptr<SPLINTER::BSpline> SplineRepresentation::buildSpline(
    const SPLINTER::DataTable &table) const {
  try {
    if (table.getNumSamples() < minSplineSamples_) {
      return nullptr;
    }

    SPLINTER::BSpline::Builder builder(table);
    builder.degree(3)
        .numBasisFunctions(numBasisFunctions_)
        .knotSpacing(SPLINTER::BSpline::KnotSpacing::EQUIDISTANT)
        .smoothing(SPLINTER::BSpline::Smoothing::NONE);

    return std::make_unique<SPLINTER::BSpline>(builder.build());
  } catch (const SPLINTER::Exception &) {
    return nullptr;
  } catch (const std::exception &) {
    return nullptr;
  }
}

} // namespace rfs

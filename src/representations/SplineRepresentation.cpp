#include "representations/SplineRepresentation.hpp"

#include <bspline.h>
#include <bsplinebuilder.h>
#include <datatable.h>

#include <algorithm>
#include <iostream>

namespace rfs {

void SplineRepresentation::update(const MeasurementSet_t &measurements) {
  if (measurements.measurements.empty()) {
    return;
  }

  std::map<double, Eigen::Vector2d> accumulated;
  std::map<double, size_t> counts;
  for (const auto &measurement : measurements.measurements) {
    accumulated[measurement.time] += measurement.value;
    ++counts[measurement.time];
  }
  for (const auto &[time, sum] : accumulated) {
    measurementHistory_[time] = sum / static_cast<double>(counts[time]);
  }
  while (measurementHistory_.size() > maxHistorySamples_) {
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
  std::cerr << "SplineRepresentation::estimate history=" << measurementHistory_.size()
            << " bsplineX=" << static_cast<bool>(bsplineX_)
            << " bsplineY=" << static_cast<bool>(bsplineY_)
            << " evalTimes=" << evaluationTimes_.size() << "\n";
  if ((!bsplineX_ || !bsplineY_) && measurementHistory_.empty()) {
    return states;
  }

  if (!bsplineX_ || !bsplineY_ || evaluationTimes_.empty()) {
    states.reserve(measurementHistory_.size());
    for (const auto &[time, position] : measurementHistory_) {
      Eigen::Vector4d state;
      state[0] = position.x();
      state[1] = position.y();
      state[2] = 0.0;
      state[3] = 0.0;
      states.push_back(state);
    }
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

    const size_t sampleCount = static_cast<size_t>(table.getNumSamples());
    const size_t basisFunctions = std::min(numBasisFunctions_, sampleCount);
    if (basisFunctions == 0) {
      return nullptr;
    }

    SPLINTER::BSpline::Builder builder(table);
    builder.degree(3)
        .numBasisFunctions(basisFunctions)
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

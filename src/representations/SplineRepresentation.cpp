#include "representations/SplineRepresentation.hpp"

#include <bspline.h>
#include <bsplinebuilder.h>
#include <datatable.h>

#include <algorithm>
#include <cmath>
#include <optional>

namespace rfs {

void SplineRepresentation::update(const MeasurementSet_t &measurements) {
  if (measurements.measurements.empty()) {
    return;
  }

  std::map<double, Eigen::Vector2d> accumulated;
  std::map<double, size_t> counts;
  for (const auto &measurement : measurements.measurements) {
    auto &sum = accumulated[measurement.time];
    if (counts[measurement.time] == 0) {
      sum = measurement.value;
    } else {
      sum += measurement.value;
    }
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
  if ((!bsplineX_ || !bsplineY_) && measurementHistory_.empty()) {
    return states;
  }

  if (!bsplineX_ || !bsplineY_ || evaluationTimes_.empty()) {
    if (measurementHistory_.empty()) {
      return states;
    }
    const auto &latest = measurementHistory_.rbegin();
    Eigen::Vector4d state;
    state[0] = latest->second.x();
    state[1] = latest->second.y();
    state[2] = 0.0;
    state[3] = 0.0;
    states.push_back(state);
    return states;
  }

  const auto isCoordinateValid = [](double value) {
    return std::isfinite(value) && std::abs(value) <= 1.0e6;
  };
  const auto fallbackState = [this]() -> std::optional<Eigen::Vector4d> {
    if (measurementHistory_.empty()) {
      return std::nullopt;
    }
    const auto &latest = measurementHistory_.rbegin();
    Eigen::Vector4d fallback;
    fallback[0] = latest->second.x();
    fallback[1] = latest->second.y();
    fallback[2] = 0.0;
    fallback[3] = 0.0;
    return fallback;
  };

  const double time = evaluationTimes_.back();
  Eigen::Vector4d state;
  bool valid = false;
  if (bsplineX_ && bsplineY_) {
    SPLINTER::DenseVector input(1);
    input[0] = time;
    try {
      state[0] = bsplineX_->eval(input);
      state[1] = bsplineY_->eval(input);
      valid = isCoordinateValid(state[0]) && isCoordinateValid(state[1]);
    } catch (const std::exception &) {
      valid = false;
    }
  }

  if (!valid) {
    if (const auto fallback = fallbackState()) {
      state = *fallback;
    } else {
      return states;
    }
  }

  state[2] = 0.0;
  state[3] = 0.0;
  states.push_back(state);

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

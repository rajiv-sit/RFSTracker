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

  for (const auto &measurement : measurements.measurements) {
    auto &history = trackHistories_[measurement.truthId];
    history.samples[measurement.time] = measurement.value;
    history.latestTime = measurement.time;

    while (history.samples.size() > maxHistorySamples_) {
      history.samples.erase(history.samples.begin());
    }

    if (history.samples.size() < minSplineSamples_) {
      history.bsplineX.reset();
      history.bsplineY.reset();
      continue;
    }

    SPLINTER::DataTable dataX(true);
    SPLINTER::DataTable dataY(true);
    for (const auto &entry : history.samples) {
      dataX.addSample(entry.first, entry.second.x());
      dataY.addSample(entry.first, entry.second.y());
    }

    history.bsplineX = buildSpline(dataX);
    history.bsplineY = buildSpline(dataY);
  }
}

std::vector<Eigen::Vector4d> SplineRepresentation::estimate() const {
  std::vector<Eigen::Vector4d> states;
  if (trackHistories_.empty()) {
    return states;
  }

  const auto isCoordinateValid = [](double value) {
    return std::isfinite(value) && std::abs(value) <= 1.0e6;
  };
  const auto fallbackState =
      [](const SplineRepresentation::TrackHistory &history) -> std::optional<Eigen::Vector4d> {
    if (history.samples.empty()) {
      return std::nullopt;
    }
    const auto &latest = history.samples.rbegin();
    Eigen::Vector4d fallback;
    fallback[0] = latest->second.x();
    fallback[1] = latest->second.y();
    fallback[2] = 0.0;
    fallback[3] = 0.0;
    return fallback;
  };

  for (const auto &[key, history] : trackHistories_) {
    if (history.samples.empty()) {
      continue;
    }

    Eigen::Vector4d state;
    bool valid = false;
    if (history.bsplineX && history.bsplineY) {
      SPLINTER::DenseVector input(1);
      input[0] = history.latestTime;
      try {
        state[0] = history.bsplineX->eval(input);
        state[1] = history.bsplineY->eval(input);
        valid = isCoordinateValid(state[0]) && isCoordinateValid(state[1]);
      } catch (const std::exception &) {
        valid = false;
      }
    }

    if (!valid) {
      if (const auto fallback = fallbackState(history)) {
        state = *fallback;
      } else {
        continue;
      }
    }

    state[2] = 0.0;
    state[3] = 0.0;
    states.push_back(state);
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

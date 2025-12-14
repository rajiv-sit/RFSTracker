#include "filters/FilterLogger.hpp"
#include "filters/RepresentationFilter.hpp"

#include <sstream>
#include <utility>

namespace rfs {

RepresentationFilter::RepresentationFilter(std::unique_ptr<IRepresentation> representation)
    : representation_(std::move(representation)) {}

void RepresentationFilter::predict(double dt) {
  std::ostringstream detail;
  detail << "dt=" << dt << " hasRepresentation=" << static_cast<bool>(representation_);
  logFilterAction("RepresentationFilter", "predict", detail.str());
  if (representation_) {
    representation_->predict(dt);
  }
}

void RepresentationFilter::update(const MeasurementSet_t &measurements) {
  std::ostringstream detail;
  detail << "measurements=" << measurements.measurements.size();
  logFilterAction("RepresentationFilter", "update", detail.str());
  if (representation_) {
    representation_->update(measurements);
  }
}

EstimatorOutput_t RepresentationFilter::estimate() const {
  EstimatorOutput_t output;
  if (!representation_) {
    return output;
  }
  const auto states = representation_->estimate();
  output.estimatedCount = static_cast<int>(states.size());
  output.tracks = states;
  return output;
}

} // namespace rfs

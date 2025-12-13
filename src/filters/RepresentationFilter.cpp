#include "filters/RepresentationFilter.hpp"

#include <utility>

namespace rfs {

RepresentationFilter::RepresentationFilter(std::unique_ptr<IRepresentation> representation)
    : representation_(std::move(representation)) {}

void RepresentationFilter::predict(double dt) {
  if (representation_) {
    representation_->predict(dt);
  }
}

void RepresentationFilter::update(const MeasurementSet_t &measurements) {
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

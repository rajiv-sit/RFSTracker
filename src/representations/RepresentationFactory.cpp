#include "representations/RepresentationFactory.hpp"

namespace rfs {

std::unique_ptr<IRepresentation>
createRepresentation(RepresentationType type) {
  using RepType = RepresentationType;
  switch (type) {
  case RepType::Particle:
    return std::make_unique<ParticleRepresentation>();
  case RepType::Spline:
    return std::make_unique<SplineRepresentation>();
  case RepType::GaussianMixture:
  default:
    return std::make_unique<GaussianRepresentation>();
  }
}

} // namespace rfs

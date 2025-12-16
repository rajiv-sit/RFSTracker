#include "representations/GaussianRepresentation.hpp"
#include "representations/ParticleRepresentation.hpp"
#include "representations/SplineRepresentation.hpp"
#include "representations/RepresentationFactory.hpp"

namespace rfs {

std::unique_ptr<IRepresentation>
createRepresentation(RepresentationType type, const TrackerConfig *config) {
  using RepType = RepresentationType;
  switch (type) {
  case RepType::Particle: {
    ParticleFilterType particleType = ParticleFilterType::SIR;
    if (config) {
      particleType = config->coreParticleType;
    }
    switch (particleType) {
    case ParticleFilterType::SIS:
      return std::make_unique<SISParticleRepresentation>();
    case ParticleFilterType::APF:
      return std::make_unique<APFParticleRepresentation>();
    case ParticleFilterType::RPF:
      return std::make_unique<RPFParticleRepresentation>();
    case ParticleFilterType::SIR:
    default:
      return std::make_unique<SIRParticleRepresentation>();
    }
  }
  case RepType::Spline:
    return std::make_unique<SplineRepresentation>();
  case RepType::GaussianMixture:
  default:
    return std::make_unique<GaussianRepresentation>();
  }
}

} // namespace rfs

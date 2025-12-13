#pragma once

#include "config/TrackerConfig.hpp"
#include "representations/GaussianRepresentation.hpp"
#include "representations/ParticleRepresentation.hpp"
#include "representations/SplineRepresentation.hpp"
#include "representations/IRepresentation.hpp"

#include <memory>

namespace rfs {

std::unique_ptr<IRepresentation>
createRepresentation(RepresentationType type);

} // namespace rfs

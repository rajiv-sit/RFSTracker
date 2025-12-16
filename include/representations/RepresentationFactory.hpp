#pragma once

#include "config/TrackerConfig.hpp"
#include "representations/IRepresentation.hpp"

#include <memory>

namespace rfs {

class TrackerConfig;

std::unique_ptr<IRepresentation>
createRepresentation(RepresentationType type, const TrackerConfig *config = nullptr);

} // namespace rfs

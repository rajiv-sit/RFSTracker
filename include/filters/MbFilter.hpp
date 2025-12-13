#pragma once

#include "filters/RepresentationFilter.hpp"

namespace rfs {

/** @brief Multi-Bernoulli filter managing Bernoulli track set. */
class MbFilter : public RepresentationFilter {
public:
  explicit MbFilter(std::unique_ptr<IRepresentation> representation);
};

} // namespace rfs

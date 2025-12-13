#pragma once

#include "filters/RepresentationFilter.hpp"

namespace rfs {

/** @brief PHD filter implementation templated on representation backend. */
class PhdFilter : public RepresentationFilter {
public:
  explicit PhdFilter(std::unique_ptr<IRepresentation> representation);
};

} // namespace rfs

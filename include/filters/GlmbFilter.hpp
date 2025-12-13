#pragma once

#include "filters/RepresentationFilter.hpp"

namespace rfs {

/** @brief GLMB filter building global hypotheses over labeled MB. */
class GlmbFilter : public RepresentationFilter {
public:
  explicit GlmbFilter(std::unique_ptr<IRepresentation> representation);
};

} // namespace rfs

#include "filters/GlmbFilter.hpp"

#include <utility>

namespace rfs {

GlmbFilter::GlmbFilter(std::unique_ptr<IRepresentation> representation)
    : RepresentationFilter(std::move(representation)) {}

} // namespace rfs

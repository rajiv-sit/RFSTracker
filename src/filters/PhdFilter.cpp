#include "filters/PhdFilter.hpp"

#include <utility>

namespace rfs {

PhdFilter::PhdFilter(std::unique_ptr<IRepresentation> representation)
    : RepresentationFilter(std::move(representation)) {}

} // namespace rfs

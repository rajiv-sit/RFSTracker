#include "filters/CphdFilter.hpp"

#include <utility>

namespace rfs {

CphdFilter::CphdFilter(std::unique_ptr<IRepresentation> representation)
    : RepresentationFilter(std::move(representation)) {}

} // namespace rfs

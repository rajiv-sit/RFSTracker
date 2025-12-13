#include "filters/MbFilter.hpp"

#include <utility>

namespace rfs {

MbFilter::MbFilter(std::unique_ptr<IRepresentation> representation)
    : RepresentationFilter(std::move(representation)) {}

} // namespace rfs

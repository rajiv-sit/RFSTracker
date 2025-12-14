#pragma once

#include "filters/RepresentationFilter.hpp"

namespace rfs {

/** @brief CPHD filter wrapping intensity and cardinality PMF logic. */
class CphdFilter : public RepresentationFilter {
public:
  explicit CphdFilter(std::unique_ptr<IRepresentation> representation);
  void predict(double dt) override;
  void update(const MeasurementSet_t &measurements) override;
};

} // namespace rfs

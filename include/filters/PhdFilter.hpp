#pragma once

#include "filters/RepresentationFilter.hpp"

namespace rfs {

/** @brief PHD filter implementation templated on representation backend. */
class PhdFilter : public RepresentationFilter {
public:
  explicit PhdFilter(std::unique_ptr<IRepresentation> representation);
  void predict(double dt) override;
  void update(const MeasurementSet_t &measurements) override;
};

} // namespace rfs

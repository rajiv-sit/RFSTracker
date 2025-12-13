#pragma once

#include "filters/IrfsFilter.hpp"
#include "representations/IRepresentation.hpp"
#include "simulation/MeasurementSet.hpp"

#include <memory>

namespace rfs {

/** @brief Filter that defers prediction/update to a representation backend. */
class RepresentationFilter : public IrfsFilter {
public:
  explicit RepresentationFilter(std::unique_ptr<IRepresentation> representation);

  void predict(double dt) override;
  void update(const MeasurementSet_t &measurements) override;
  EstimatorOutput_t estimate() const override;

protected:
  std::unique_ptr<IRepresentation> representation_;
};

} // namespace rfs

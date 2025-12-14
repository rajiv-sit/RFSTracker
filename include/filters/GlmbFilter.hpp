#pragma once

#include "filters/MbFilter.hpp"

namespace rfs {

class GlmbFilter : public MbFilter {
public:
  explicit GlmbFilter(RepresentationType representationType,
                      const TrackerConfig *config);

  void update(const MeasurementSet_t &measurements) override;
  EstimatorOutput_t estimate() const override;

private:
  double globalWeight_ = 1.0;
};

} // namespace rfs

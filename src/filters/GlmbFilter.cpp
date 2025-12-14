#include "filters/GlmbFilter.hpp"

#include <algorithm>

namespace rfs {

GlmbFilter::GlmbFilter(RepresentationType representationType,
                       const TrackerConfig *config)
    : MbFilter(representationType, config) {}

void GlmbFilter::update(const MeasurementSet_t &measurements) {
  MbFilter::update(measurements);
  double existenceSum = 0.0;
  for (const auto &hypothesis : hypotheses()) {
    existenceSum += hypothesis.existence;
  }
  globalWeight_ = 0.95 * globalWeight_ + 0.05 * existenceSum;
}

EstimatorOutput_t GlmbFilter::estimate() const {
  auto output = MbFilter::estimate();
  output.nees = globalWeight_;
  return output;
}

} // namespace rfs

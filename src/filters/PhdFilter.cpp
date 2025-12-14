#include "filters/FilterLogger.hpp"
#include "filters/PhdFilter.hpp"

#include <sstream>

namespace rfs {

PhdFilter::PhdFilter(std::unique_ptr<IRepresentation> representation)
    : RepresentationFilter(std::move(representation)) {}

void PhdFilter::predict(double dt) {
  std::ostringstream detail;
  detail << "dt=" << dt;
  logFilterAction("PhdFilter", "predict", detail.str());
  RepresentationFilter::predict(dt);
}

void PhdFilter::update(const MeasurementSet_t &measurements) {
  std::ostringstream detail;
  detail << "measurements=" << measurements.measurements.size();
  logFilterAction("PhdFilter", "update", detail.str());
  RepresentationFilter::update(measurements);
}

} // namespace rfs

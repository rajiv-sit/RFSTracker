#include "filters/CphdFilter.hpp"
#include "filters/FilterLogger.hpp"

#include <sstream>

namespace rfs {

CphdFilter::CphdFilter(std::unique_ptr<IRepresentation> representation)
    : RepresentationFilter(std::move(representation)) {}

void CphdFilter::predict(double dt) {
  std::ostringstream detail;
  detail << "dt=" << dt;
  logFilterAction("CphdFilter", "predict", detail.str());
  RepresentationFilter::predict(dt);
}

void CphdFilter::update(const MeasurementSet_t &measurements) {
  std::ostringstream detail;
  detail << "measurements=" << measurements.measurements.size();
  logFilterAction("CphdFilter", "update", detail.str());
  RepresentationFilter::update(measurements);
}

} // namespace rfs

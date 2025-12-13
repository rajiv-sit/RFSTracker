#include "numerics/NumericsUtils.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rfs {

double NumericsUtils::logSumExp(const std::vector<double> &values) {
  if (values.empty()) {
    return -std::numeric_limits<double>::infinity();
  }

  const double maxValue = *std::max_element(values.begin(), values.end());
  double sum = 0.0;
  for (double each : values) {
    sum += std::exp(each - maxValue);
  }
  return maxValue + std::log(sum);
}

void NumericsUtils::stableNormalize(std::vector<double> &values) {
  if (values.empty()) {
    return;
  }
  const double logSum = logSumExp(values);
  for (double &value : values) {
    value = std::exp(value - logSum);
  }
}

} // namespace rfs

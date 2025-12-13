#pragma once

#include <vector>

namespace rfs {

/** @brief Utility helpers for numerics such as log-sum-exp and normalization. */
class NumericsUtils {
public:
  static double logSumExp(const std::vector<double> &values);

  static void stableNormalize(std::vector<double> &values);
};

} // namespace rfs

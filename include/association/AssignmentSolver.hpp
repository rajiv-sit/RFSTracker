#pragma once

#include <vector>

namespace rfs {

using CostMatrix_t = std::vector<std::vector<double>>;

struct AssociationResult_t {
  std::vector<int> assignment;
  double totalCost = 0.0;
};

/** @brief Solver interface for data association. */
class AssignmentSolver {
public:
  virtual ~AssignmentSolver() = default;
  virtual AssociationResult_t solve(const CostMatrix_t &costMatrix) = 0;
};

} // namespace rfs

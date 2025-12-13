#pragma once

#include "AssignmentSolver.hpp"

namespace rfs {

/** @brief Eigen-based port of the MATLAB Hungarian solver. */
class HungarianSolver : public AssignmentSolver {
public:
  AssociationResult_t solve(const CostMatrix_t &costMatrix) override;
};

} // namespace rfs

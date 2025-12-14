#include "association/AssociationLogger.hpp"
#include "association/HungarianSolver.hpp"

#include "association/Hungarian.hpp"
#include <algorithm>
#include <Eigen/Dense>
#include <sstream>

namespace rfs {

AssociationResult_t HungarianSolver::solve(const CostMatrix_t &costMatrix) {
  AssociationResult_t result;
  if (costMatrix.empty() || costMatrix.front().empty()) {
    return result;
  }

  const size_t rows = costMatrix.size();
  const size_t cols = costMatrix.front().size();
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> matrix(rows, cols);
  for (size_t i = 0; i < rows; ++i) {
    for (size_t j = 0; j < cols; ++j) {
      matrix(i, j) = costMatrix[i][j];
    }
  }

  Hungarian<double> solver(matrix);
  const auto assignmentVec = solver.assignmentZeroIndexed();

  result.assignment.assign(rows, -1);
  const size_t assignSize = std::min(rows, assignmentVec.size());
  for (size_t i = 0; i < assignSize; ++i) {
    const auto columnId = assignmentVec[i];
    if (columnId >= 0 && columnId < static_cast<int>(cols)) {
      result.assignment[i] = columnId;
    }
  }

  result.totalCost = static_cast<double>(solver.getOverAllCost());

  std::ostringstream detail;
  detail << "HungarianSolver::solve matrix=" << rows << "x" << cols << " assignment=[";
  for (size_t i = 0; i < assignmentVec.size(); ++i) {
    if (i > 0) {
      detail << ",";
    }
    detail << assignmentVec[i];
  }
  detail << "] totalCost=" << result.totalCost;
  logAssociation(detail.str());
  return result;
}

} // namespace rfs

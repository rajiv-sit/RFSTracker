#include "association/HungarianSolver.hpp"

#include <algorithm>
#include <limits>

namespace rfs {

AssociationResult_t HungarianSolver::solve(const CostMatrix_t &costMatrix) {
  AssociationResult_t result;
  if (costMatrix.empty() || costMatrix.front().empty()) {
    return result;
  }

  const size_t rows = costMatrix.size();
  const size_t cols = costMatrix.front().size();
  const size_t n = std::max(rows, cols);

  std::vector<std::vector<double>> matrix(n + 1, std::vector<double>(n + 1, 0.0));
  double maxCost = 0.0;
  for (const auto &row : costMatrix) {
    for (double value : row) {
      maxCost = std::max(maxCost, value);
    }
  }
  const double padCost = maxCost * 10.0 + 1.0;

  for (size_t i = 1; i <= n; ++i) {
    for (size_t j = 1; j <= n; ++j) {
      if (i <= rows && j <= cols) {
        matrix[i][j] = costMatrix[i - 1][j - 1];
      } else {
        matrix[i][j] = padCost;
      }
    }
  }

  std::vector<double> u(n + 1, 0.0);
  std::vector<double> v(n + 1, 0.0);
  std::vector<int> p(n + 1, 0);
  std::vector<int> way(n + 1, 0);

  for (int i = 1; i <= static_cast<int>(n); ++i) {
    p[0] = i;
    int j0 = 0;
    std::vector<double> minv(n + 1, std::numeric_limits<double>::infinity());
    std::vector<char> used(n + 1, false);
    do {
      used[j0] = true;
      const int i0 = p[j0];
      double delta = std::numeric_limits<double>::infinity();
      int j1 = 0;
      for (int j = 1; j <= static_cast<int>(n); ++j) {
        if (used[j]) {
          continue;
        }
        const double cur = matrix[i0][j] - u[i0] - v[j];
        if (cur < minv[j]) {
          minv[j] = cur;
          way[j] = j0;
        }
        if (minv[j] < delta) {
          delta = minv[j];
          j1 = j;
        }
      }
      for (int j = 0; j <= static_cast<int>(n); ++j) {
        if (used[j]) {
          u[p[j]] += delta;
          v[j] -= delta;
        } else {
          minv[j] -= delta;
        }
      }
      j0 = j1;
    } while (p[j0] != 0);
    do {
      int j1 = way[j0];
      p[j0] = p[j1];
      j0 = j1;
    } while (j0 != 0);
  }

  result.assignment.assign(rows, -1);
  for (int j = 1; j <= static_cast<int>(n); ++j) {
    if (p[j] != 0 && p[j] <= static_cast<int>(rows) && j <= static_cast<int>(cols)) {
      result.assignment[p[j] - 1] = j - 1;
    }
  }

  double costSum = 0.0;
  for (size_t i = 0; i < rows; ++i) {
    if (result.assignment[i] >= 0) {
      costSum += costMatrix[i][result.assignment[i]];
    } else {
      costSum += padCost;
    }
  }
  result.totalCost = costSum;
  return result;
}

} // namespace rfs

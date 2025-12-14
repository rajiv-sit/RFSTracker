#pragma once

#include <Eigen/Dense>
#include <algorithm>
#include <limits>
#include <type_traits>
#include <utility>
#include <vector>

namespace rfs {

template <typename T>
class Hungarian {
  static_assert(std::is_floating_point_v<T>, "Hungarian solver requires floating-point types.");

protected:
  struct Assignment_t {
    Eigen::Vector<uint32_t, Eigen::Dynamic> assignment;
    T cost;
  };

  Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> mCostMatrix;
  Assignment_t mResult{};
  bool mStatus{false};
  static constexpr T kPaddedCost =
      std::numeric_limits<T>::max() / static_cast<T>(4); // large but finite sentinel; used only for padding helper rows and columns

public:
  Hungarian() = default;
  explicit Hungarian(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &costMatrix)
      : mCostMatrix(costMatrix) {
    if (mCostMatrix.rows() > 0 && mCostMatrix.cols() > 0) {
      mResult = process(mCostMatrix);
      mStatus = (mResult.assignment.rows() > 0);
    }
  }

  ~Hungarian() = default;

  Eigen::Vector<uint32_t, Eigen::Dynamic> getAssignmentIDs() const {
    return mResult.assignment;
  }

  T getOverAllCost() const { return mResult.cost; }

  bool getStatus() const { return mStatus; }

  std::vector<int> assignmentZeroIndexed() const {
    std::vector<int> result;
    result.reserve(mResult.assignment.rows());
    for (int idx = 0; idx < mResult.assignment.rows(); ++idx) {
      const auto value = mResult.assignment(idx);
      result.push_back(value > 0 ? static_cast<int>(value - 1) : -1);
    }
    return result;
  }

private:
  struct SolverResult {
    std::vector<int> assignment;
  };

  Assignment_t process(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &costMat) {
    const int rows = static_cast<int>(costMat.rows());
    const int cols = static_cast<int>(costMat.cols());
    const int n = std::max(rows, cols);

    Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> padded =
        Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>::Constant(n, n, kPaddedCost);
    for (int r = 0; r < rows; ++r) {
      for (int c = 0; c < cols; ++c) {
        padded(r, c) = costMat(r, c);
      }
    }

    const auto solverResult = solveSquare(padded);
    Eigen::Vector<uint32_t, Eigen::Dynamic> assignment =
        Eigen::Vector<uint32_t, Eigen::Dynamic>::Zero(rows);
    T totalCost = T(0);

    for (int r = 0; r < rows; ++r) {
      const int col = solverResult.assignment.at(r + 1);
      if (col > 0) {
        assignment(r) = static_cast<uint32_t>(col);
        if (col <= cols) {
          totalCost += costMat(r, col - 1);
        }
      }
    }

    return {assignment, totalCost};
  }

  SolverResult solveSquare(const Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> &matrix) const {
    const int n = static_cast<int>(matrix.rows());
    std::vector<T> u(n + 1, T(0));
    std::vector<T> v(n + 1, T(0));
    std::vector<int> p(n + 1, 0);
    std::vector<int> way(n + 1, 0);
    const T maxCost = std::numeric_limits<T>::max();

    for (int i = 1; i <= n; ++i) {
      p[0] = i;
      int j0 = 0;
      std::vector<T> minv(n + 1, maxCost);
      minv[0] = 0;
      std::vector<char> used(n + 1, 0);
      do {
        used[j0] = 1;
        const int i0 = p[j0];
        T delta = maxCost;
        int j1 = 0;
        for (int j = 1; j <= n; ++j) {
          if (used[j]) {
            continue;
          }
          const T cur = matrix(i0 - 1, j - 1) - u[i0] - v[j];
          if (cur < minv[j]) {
            minv[j] = cur;
            way[j] = j0;
          }
          if (minv[j] < delta) {
            delta = minv[j];
            j1 = j;
          }
        }
        for (int j = 0; j <= n; ++j) {
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
        const int j1 = way[j0];
        p[j0] = p[j1];
        j0 = j1;
      } while (j0 != 0);
    }

    std::vector<int> assignment(n + 1, 0);
    for (int j = 1; j <= n; ++j) {
      assignment[p[j]] = j;
    }

    return {assignment};
  }
};

} // namespace rfs

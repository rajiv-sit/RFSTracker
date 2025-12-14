#pragma once

#include "representations/IRepresentation.hpp"

#include <Eigen/Dense>
#include <optional>
#include <vector>

namespace rfs {

struct GaussianComponent_t {
  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Identity();
  double weight = 1.0;
  int hits = 0;
};

/** @brief Gaussian mixture helper utilities. */
class GaussianRepresentation : public IRepresentation {
public:
  void predict(double dt) override;
  void update(const MeasurementSet_t &measurements) override;
  std::vector<Eigen::Vector4d> estimate() const override;

  void addComponent(const GaussianComponent_t &component);
  void prune(double weightThreshold = 1e-3);
  void merge();

  const std::vector<GaussianComponent_t> &components() const;

private:
  std::vector<GaussianComponent_t> components_;
  std::optional<Eigen::Vector4d> lastEstimate_;
  double adaptiveGate_ = 5.0;
  int consecutiveMisses_ = 0;
};

} // namespace rfs

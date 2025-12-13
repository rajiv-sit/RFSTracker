#include "representations/GaussianRepresentation.hpp"

#include <algorithm>
#include <cmath>

namespace rfs {

void GaussianRepresentation::predict(double dt) {
  for (auto &component : components_) {
    component.mean[0] += component.mean[2] * dt;
    component.mean[1] += component.mean[3] * dt;
    component.covariance += Eigen::Matrix4d::Identity() * 0.01;
  }
}

void GaussianRepresentation::update(const MeasurementSet_t &measurements) {
  components_.clear();
  components_.reserve(measurements.measurements.size());

  for (const auto &measurement : measurements.measurements) {
    GaussianComponent_t component;
    component.mean.head<2>() = measurement.value;
    component.mean[2] = 0.0;
    component.mean[3] = 0.0;
    component.covariance = Eigen::Matrix4d::Identity();
    component.weight = 1.0;
    components_.push_back(component);
  }

  prune();
  merge();
}

std::vector<Eigen::Vector4d> GaussianRepresentation::estimate() const {
  std::vector<Eigen::Vector4d> result;
  result.reserve(components_.size());
  for (const auto &component : components_) {
    result.push_back(component.mean);
  }
  return result;
}

void GaussianRepresentation::addComponent(const GaussianComponent_t &component) {
  components_.push_back(component);
}

void GaussianRepresentation::prune(double weightThreshold) {
  components_.erase(
      std::remove_if(components_.begin(), components_.end(),
                     [weightThreshold](const GaussianComponent_t &component) {
                       return component.weight < weightThreshold;
                     }),
      components_.end());
}

void GaussianRepresentation::merge() {
  if (components_.empty()) {
    return;
  }

  Eigen::Vector4d mean = Eigen::Vector4d::Zero();
  Eigen::Matrix4d covariance = Eigen::Matrix4d::Zero();
  double totalWeight = 0.0;

  for (const auto &component : components_) {
    mean += component.weight * component.mean;
    totalWeight += component.weight;
  }

  if (totalWeight <= 0.0) {
    return;
  }

  mean /= totalWeight;

  for (const auto &component : components_) {
    const Eigen::Vector4d delta = component.mean - mean;
    covariance += component.weight *
                  (component.covariance + delta * delta.transpose());
  }

  covariance /= totalWeight;
  components_.clear();
  components_.push_back({mean, covariance, totalWeight});
}

const std::vector<GaussianComponent_t> &GaussianRepresentation::components() const {
  return components_;
}

} // namespace rfs

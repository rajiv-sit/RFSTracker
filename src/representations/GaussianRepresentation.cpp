#include "representations/GaussianRepresentation.hpp"

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <sstream>
#include <string>
#include "logging/LoggerControl.hpp"
#include "representations/RepresentationLogger.hpp"

namespace rfs {

namespace {
constexpr double kGatingRadius = 20.0;
constexpr double kBaseGatingRadius = 20.0;
constexpr double kMaxAdaptiveGate = 40.0;
constexpr double kGatePromotion = 5.0;
constexpr double kWeightDecay = 0.2;
struct GaussianLogWriter {
  void logLine(const std::string &line) {
    if (!loggerVerboseEnabled()) {
      return;
    }
    ensureStream();
    stream << line << '\n';
    stream.flush();
  }

 private:
  void ensureStream() {
    if (!stream.is_open()) {
      stream.open("representation_debug.log", std::ios::trunc);
    }
  }

  std::ofstream stream;
};

GaussianLogWriter &logger() {
  static GaussianLogWriter instance;
  return instance;
}

std::string formatComponent(const GaussianComponent_t &component) {
  std::ostringstream oss;
  const auto &mean = component.mean;
  oss << "mean=(" << std::fixed << std::setprecision(2) << mean.x() << "," << mean.y() << ","
      << mean.z() << "," << mean.w() << ")";
  oss << " weight=" << std::fixed << std::setprecision(2) << component.weight;
  oss << " covDiag=(" << component.covariance(0, 0) << "," << component.covariance(1, 1) << ","
      << component.covariance(2, 2) << "," << component.covariance(3, 3) << ")";
  return oss.str();
}

std::string formatMeasurement(const Measurement_t &measurement) {
  std::ostringstream oss;
  oss << "t=" << std::fixed << std::setprecision(2) << measurement.time;
  oss << " p=(" << std::fixed << std::setprecision(2) << measurement.value.x() << ","
      << measurement.value.y() << ")";
  oss << " truth=";
  if (measurement.truthId) {
    oss << *measurement.truthId;
  } else {
    oss << "clutter";
  }
  return oss.str();
}

std::string scanPrefix() {
  std::ostringstream oss;
  oss << "[scan " << representationScanId() << "]";
  return oss.str();
}
} // namespace

void GaussianRepresentation::predict(double dt) {
  {
    std::ostringstream line;
    line << scanPrefix() << " [GaussianRepresentation::predict] componentCount=" << components_.size();
    logger().logLine(line.str());
  }
  for (auto &component : components_) {
    component.mean[0] += component.mean[2] * dt;
    component.mean[1] += component.mean[3] * dt;
    component.covariance += Eigen::Matrix4d::Identity() * 0.01;
  }
}

double distanceToComponent(const GaussianComponent_t &component, const Eigen::Vector2d &point) {
  return (component.mean.head<2>() - point).norm();
}

void GaussianRepresentation::update(const MeasurementSet_t &measurements) {
  std::vector<GaussianComponent_t> updatedComponents = components_;
  std::vector<bool> touched(updatedComponents.size(), false);
  std::vector<Measurement_t> gatedMeasurements;
  gatedMeasurements.reserve(measurements.measurements.size());
  double currentGate = std::max(kBaseGatingRadius, adaptiveGate_);
  bool anyMatched = false;

  for (const auto &measurement : measurements.measurements) {
    double bestDist = std::numeric_limits<double>::max();
    std::optional<size_t> bestIdx;
    for (size_t idx = 0; idx < updatedComponents.size(); ++idx) {
      const double dist = distanceToComponent(updatedComponents[idx], measurement.value);
      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = idx;
      }
    }
    if (bestIdx && bestDist <= currentGate) {
      auto &component = updatedComponents[*bestIdx];
      Eigen::Vector2d old = component.mean.head<2>();
      component.mean.head<2>() = (old + measurement.value) * 0.5;
      component.mean[2] = (component.mean[2] + 0.0) * 0.5;
      component.mean[3] = (component.mean[3] + 0.0) * 0.5;
      component.covariance = Eigen::Matrix4d::Identity();
      component.weight += 1.0;
      component.hits = std::max(1, component.hits + 1);
      touched[*bestIdx] = true;
      gatedMeasurements.push_back(measurement);
      anyMatched = true;
      std::ostringstream logLine;
      logLine << scanPrefix() << " [GaussianRepresentation::update] measurement matched component[" << *bestIdx
              << "] dist=" << std::fixed << std::setprecision(2) << bestDist;
      logger().logLine(logLine.str());
      continue;
    }
    GaussianComponent_t newComponent;
    newComponent.mean.head<2>() = measurement.value;
    newComponent.mean[2] = 0.0;
    newComponent.mean[3] = 0.0;
    newComponent.covariance = Eigen::Matrix4d::Identity();
    newComponent.weight = 1.0;
    newComponent.hits = 1;
    updatedComponents.push_back(newComponent);
    touched.push_back(true);
    gatedMeasurements.push_back(measurement);
    std::ostringstream logLine;
    logLine << scanPrefix()
            << " [GaussianRepresentation::update] measurement started new component dist=N/A";
    logger().logLine(logLine.str());
  }

  if (!anyMatched) {
    consecutiveMisses_++;
    adaptiveGate_ = std::min(adaptiveGate_ + kGatePromotion, kMaxAdaptiveGate);
    std::ostringstream line;
    line << scanPrefix()
         << " [GaussianRepresentation::update] no measurements matched; adaptiveGate=" << adaptiveGate_;
    logger().logLine(line.str());
  } else {
    consecutiveMisses_ = 0;
    adaptiveGate_ = kBaseGatingRadius;
  }

  for (size_t idx = 0; idx < updatedComponents.size(); ++idx) {
    if (!touched[idx]) {
      updatedComponents[idx].weight = std::max(0.0, updatedComponents[idx].weight - kWeightDecay);
      updatedComponents[idx].hits = std::max(0, updatedComponents[idx].hits - 1);
    }
  }

  {
    std::ostringstream summary;
    summary << scanPrefix()
            << " [GaussianRepresentation::update] measurementCount=" << measurements.measurements.size()
            << " gated=" << gatedMeasurements.size();
    for (const auto &measurement : gatedMeasurements) {
      summary << " {" << formatMeasurement(measurement) << "}";
    }
    logger().logLine(summary.str());
  }

  prune(updatedComponents, 1e-3);
  merge(updatedComponents);

  components_ = std::move(updatedComponents);

  if (components_.empty()) {
    logger().logLine(scanPrefix() + " [GaussianRepresentation::update] no components after merge");
  } else {
    for (size_t idx = 0; idx < components_.size(); ++idx) {
      std::ostringstream line;
      line << scanPrefix() << " [GaussianRepresentation::update] component[" << idx << "] "
           << formatComponent(components_[idx]);
      logger().logLine(line.str());
    }
  }
  if (!components_.empty()) {
    auto bestIt = std::max_element(
        components_.begin(), components_.end(),
        [](const GaussianComponent_t &a, const GaussianComponent_t &b) {
          return a.weight < b.weight;
        });
    lastEstimate_ = bestIt->mean;
  }
}

std::vector<Eigen::Vector4d> GaussianRepresentation::estimate() const {
  if (components_.empty()) {
    logger().logLine(scanPrefix() + " [GaussianRepresentation::estimate] no components to report");
  } else {
    std::ostringstream summary;
    summary << scanPrefix() << " [GaussianRepresentation::estimate] componentCount=" << components_.size();
    logger().logLine(summary.str());
    for (size_t idx = 0; idx < components_.size(); ++idx) {
      std::ostringstream line;
      line << scanPrefix() << " [GaussianRepresentation::estimate] component[" << idx << "] "
           << formatComponent(components_[idx]);
      logger().logLine(line.str());
    }
  }
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
  prune(components_, weightThreshold);
}

void GaussianRepresentation::prune(std::vector<GaussianComponent_t> &components,
                                    double weightThreshold) {
  components.erase(
      std::remove_if(components.begin(), components.end(),
                     [weightThreshold](const GaussianComponent_t &component) {
                       return component.weight < weightThreshold || component.hits <= 0;
                     }),
      components.end());
}

void GaussianRepresentation::merge() {
  merge(components_);
}

void GaussianRepresentation::merge(std::vector<GaussianComponent_t> &components) {
  if (components.size() <= 1) {
    return;
  }

  std::vector<bool> visited(components.size(), false);
  std::vector<GaussianComponent_t> mergedComponents;
  for (size_t i = 0; i < components.size(); ++i) {
    if (visited[i]) {
      continue;
    }
    GaussianComponent_t aggregate = components[i];
    double totalWeight = aggregate.weight;
    int totalHits = aggregate.hits;
    Eigen::Vector4d mean = aggregate.mean * aggregate.weight;
    Eigen::Matrix4d covariance = aggregate.covariance * aggregate.weight;
    visited[i] = true;

    for (size_t j = i + 1; j < components.size(); ++j) {
      if (visited[j]) {
        continue;
      }
      const double dist = distanceToComponent(components[j], aggregate.mean.head<2>());
      if (dist <= kGatingRadius) {
        const auto &comp = components[j];
        visited[j] = true;
        totalWeight += comp.weight;
        totalHits += comp.hits;
        mean += comp.mean * comp.weight;
        covariance += comp.covariance * comp.weight;
      }
    }

    if (totalWeight > 0.0) {
      aggregate.mean = mean / totalWeight;
      aggregate.covariance = covariance / totalWeight;
      aggregate.weight = totalWeight;
      aggregate.hits = totalHits;
    }
    mergedComponents.push_back(aggregate);
  }

  components.swap(mergedComponents);
}

const std::vector<GaussianComponent_t> &GaussianRepresentation::components() const {
  return components_;
}

} // namespace rfs

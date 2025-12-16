#include "representations/ParticleRepresentation.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <utility>

namespace rfs {

namespace {
constexpr double kProcessNoiseStd = 0.1;
constexpr double kMeasurementStd = 5.0;
constexpr size_t kDefaultParticleCount = 200;
constexpr double kInitialSpread = 20.0;
constexpr double kVelocitySpread = 1.0;
constexpr double kEstimateClusterRadius = 30.0;
constexpr size_t kMaxEstimates = 20;
constexpr double kDetectionAssociationRadius = 15.0;
} // namespace

ParticleRepresentationBase::ParticleRepresentationBase()
    : rng_(std::random_device{}()) {}

void ParticleRepresentationBase::predict(double dt) {
  std::normal_distribution<double> processNoise(0.0, kProcessNoiseStd);
  for (auto &particle : particles_) {
    particle.state[0] += particle.state[2] * dt + processNoise(rng_);
    particle.state[1] += particle.state[3] * dt + processNoise(rng_);
  }
}

std::vector<Eigen::Vector4d> ParticleRepresentationBase::estimate() const {
  std::vector<Eigen::Vector4d> estimates;
  if (particles_.empty()) {
    return estimates;
  }

  if (!trackedDetectionPositions_.empty()) {
    std::vector<bool> consumed(particles_.size(), false);
    for (const auto &position : trackedDetectionPositions_) {
      if (estimates.size() >= kMaxEstimates) {
        break;
      }
      Eigen::Vector4d centroid = Eigen::Vector4d::Zero();
      double totalWeight = 0.0;
      for (size_t j = 0; j < particles_.size(); ++j) {
        if (consumed[j]) {
          continue;
        }
        const Eigen::Vector2d pos = particles_[j].state.head<2>();
        if ((pos - position).norm() <= kEstimateClusterRadius) {
          centroid += particles_[j].state * particles_[j].weight;
          totalWeight += particles_[j].weight;
          consumed[j] = true;
        }
      }
      if (totalWeight <= 0.0) {
        centroid << position.x(), position.y(), 0.0, 0.0;
      } else {
        centroid /= totalWeight;
      }
      estimates.push_back(centroid);
    }
    if (!estimates.empty()) {
      return estimates;
    }
  }

  std::vector<size_t> indices(particles_.size());
  std::iota(indices.begin(), indices.end(), 0);
  std::sort(indices.begin(), indices.end(),
            [this](size_t a, size_t b) { return particles_[a].weight > particles_[b].weight; });

  std::vector<bool> consumed(particles_.size(), false);
  for (size_t idx : indices) {
    if (consumed[idx]) {
      continue;
    }
    Eigen::Vector4d centroid = Eigen::Vector4d::Zero();
    double totalWeight = 0.0;
    for (size_t j = 0; j < particles_.size(); ++j) {
      if (consumed[j]) {
        continue;
      }
      const Eigen::Vector2d pos = particles_[j].state.head<2>();
      const Eigen::Vector2d reference = particles_[idx].state.head<2>();
      if ((pos - reference).norm() <= kEstimateClusterRadius) {
        centroid += particles_[j].state * particles_[j].weight;
        totalWeight += particles_[j].weight;
        consumed[j] = true;
      }
    }
    if (totalWeight <= 0.0) {
      centroid = particles_[idx].state;
    } else {
      centroid /= totalWeight;
    }
    estimates.push_back(centroid);
    if (estimates.size() >= kMaxEstimates) {
      break;
    }
  }

  return estimates;
}

void ParticleRepresentationBase::setParticles(std::vector<Particle_t> particles) {
  particles_ = std::move(particles);
}

const std::vector<Particle_t> &ParticleRepresentationBase::particles() const {
  return particles_;
}

void ParticleRepresentationBase::ensureInitialized(
    const MeasurementSet_t &measurements) {
  if (measurements.measurements.empty()) {
    trackedDetectionPositions_.clear();
    return;
  }

  std::vector<const Measurement_t *> detectionMeasurements;
  detectionMeasurements.reserve(measurements.measurements.size());
  for (const auto &measurement : measurements.measurements) {
    if (measurement.isClutter) {
      continue;
    }
    detectionMeasurements.push_back(&measurement);
  }
  if (detectionMeasurements.empty()) {
    trackedDetectionPositions_.clear();
    return;
  }

  if (particles_.empty()) {
    const size_t detectionCount = detectionMeasurements.size();
    const size_t perMeasurement =
        std::max<size_t>(1, kDefaultParticleCount / detectionCount);
    particles_.reserve(kDefaultParticleCount);

    for (const auto *measurement : detectionMeasurements) {
      spawnParticles(*measurement, perMeasurement);
    }

    if (particles_.size() < kDefaultParticleCount) {
      for (size_t idx = particles_.size(); idx < kDefaultParticleCount;
           ++idx) {
        spawnParticles(*detectionMeasurements[idx % detectionCount], 1);
      }
    }

    recordDetectionPositions(detectionMeasurements);
    std::vector<double> uniform(particles_.size(), 1.0);
    applyWeights(uniform);
    return;
  }

  const auto newDetections =
      collectNewDetectionMeasurements(detectionMeasurements);
  if (newDetections.empty()) {
    recordDetectionPositions(detectionMeasurements);
    return;
  }

  const size_t perMeasurement =
      std::max<size_t>(1, kDefaultParticleCount / newDetections.size());
  for (const auto *measurement : newDetections) {
    spawnParticles(*measurement, perMeasurement);
  }
  recordDetectionPositions(detectionMeasurements);
  std::vector<double> uniform(particles_.size(), 1.0);
  applyWeights(uniform);
}

size_t ParticleRepresentationBase::particleCount() const {
  return particles_.size();
}

double ParticleRepresentationBase::measurementDenominator() const {
  return 2.0 * kMeasurementStd * kMeasurementStd;
}

std::vector<double> ParticleRepresentationBase::computeMeasurementWeights(
    const MeasurementSet_t &measurements, double denom) const {
  std::vector<double> weights;
  weights.reserve(particles_.size());
  for (const auto &particle : particles_) {
    weights.push_back(measurementLikelihood(particle, measurements, denom));
  }
  return weights;
}

std::vector<double> ParticleRepresentationBase::computeAuxiliaryWeights(
    const MeasurementSet_t &measurements, double denom) const {
  std::vector<double> weights;
  weights.reserve(particles_.size());
  for (const auto &particle : particles_) {
    double score = 0.0;
    const auto statePos = particle.state.head<2>();
    for (const auto &measurement : measurements.measurements) {
      const double distance = (statePos - measurement.value).norm();
      score += std::exp(-distance * distance / denom);
    }
    weights.push_back(std::max(1e-6, score) * particle.weight);
  }
  return weights;
}

void ParticleRepresentationBase::applyWeights(std::vector<double> &weights) {
  if (weights.empty() || particles_.empty()) {
    return;
  }
  normalizeWeights(weights);
  for (size_t i = 0; i < weights.size() && i < particles_.size(); ++i) {
    particles_[i].weight = weights[i];
  }
}

void ParticleRepresentationBase::resampleWithWeights(
    const std::vector<double> &weights) {
  if (particles_.empty() || particles_.size() != weights.size()) {
    return;
  }
  std::vector<double> normalized = weights;
  normalizeWeights(normalized);
  std::discrete_distribution<size_t> dist(normalized.begin(), normalized.end());
  std::vector<Particle_t> resampled;
  resampled.reserve(particles_.size());
  for (size_t i = 0; i < particles_.size(); ++i) {
    resampled.push_back(particles_[dist(rng_)]);
    resampled.back().weight = 1.0 / particles_.size();
  }
  particles_ = std::move(resampled);
}

void ParticleRepresentationBase::resample() {
  if (particles_.empty()) {
    return;
  }
  std::vector<double> weights;
  weights.reserve(particles_.size());
  for (const auto &particle : particles_) {
    weights.push_back(particle.weight);
  }
  resampleWithWeights(weights);
}

void ParticleRepresentationBase::regularize(double stddev) {
  if (particles_.empty()) {
    return;
  }
  std::normal_distribution<double> jitter(0.0, stddev);
  for (auto &particle : particles_) {
    particle.state[0] += jitter(rng_);
    particle.state[1] += jitter(rng_);
    particle.state[2] += jitter(rng_) * 0.5;
    particle.state[3] += jitter(rng_) * 0.5;
  }
}

void ParticleRepresentationBase::normalizeWeights(std::vector<double> &weights) const {
  double sum = std::accumulate(weights.begin(), weights.end(), 0.0);
  if (sum <= 0.0) {
    const double uniformWeight = 1.0 / std::max<size_t>(weights.size(), 1);
    for (double &value : weights) {
      value = uniformWeight;
    }
    return;
  }
  for (double &value : weights) {
    value /= sum;
  }
}

double ParticleRepresentationBase::measurementLikelihood(
    const Particle_t &particle, const MeasurementSet_t &measurements,
    double denom) const {
  double likelihood = 0.0;
  const auto statePos = particle.state.head<2>();
  for (const auto &measurement : measurements.measurements) {
    const double distance = (statePos - measurement.value).norm();
    likelihood += std::exp(-distance * distance / denom);
  }
  likelihood += 1e-6;
  return likelihood;
}

void ParticleRepresentationBase::spawnParticles(
    const Measurement_t &measurement, size_t count) {
  if (count == 0) {
    return;
  }
  std::normal_distribution<double> positionNoise(0.0, kInitialSpread);
  std::normal_distribution<double> velocityNoise(0.0, kVelocitySpread);
  const auto basePosition = measurement.value;
  for (size_t idx = 0; idx < count; ++idx) {
    Particle_t particle;
    particle.state[0] = basePosition[0] + positionNoise(rng_);
    particle.state[1] = basePosition[1] + positionNoise(rng_);
    particle.state[2] = velocityNoise(rng_);
    particle.state[3] = velocityNoise(rng_);
    particle.weight = 1.0;
    particles_.push_back(particle);
  }
}

std::vector<const Measurement_t *>
ParticleRepresentationBase::collectNewDetectionMeasurements(
    const std::vector<const Measurement_t *> &detections) const {
  std::vector<const Measurement_t *> newDetections;
  newDetections.reserve(detections.size());
  for (const auto *measurement : detections) {
    const auto position = measurement->value;
    bool associated = false;
    for (const auto &tracked : trackedDetectionPositions_) {
      if ((tracked - position).norm() <= kDetectionAssociationRadius) {
        associated = true;
        break;
      }
    }
    if (!associated) {
      newDetections.push_back(measurement);
    }
  }
  return newDetections;
}

void ParticleRepresentationBase::recordDetectionPositions(
    const std::vector<const Measurement_t *> &detections) {
  trackedDetectionPositions_.clear();
  trackedDetectionPositions_.reserve(detections.size());
  for (const auto *measurement : detections) {
    trackedDetectionPositions_.push_back(measurement->value);
  }
}

void SIRParticleRepresentation::update(const MeasurementSet_t &measurements) {
  ensureInitialized(measurements);
  if (particleCount() == 0 || measurements.measurements.empty()) {
    return;
  }
  const double denom = measurementDenominator();
  auto weights = computeMeasurementWeights(measurements, denom);
  applyWeights(weights);
  resample();
}

void SISParticleRepresentation::update(const MeasurementSet_t &measurements) {
  ensureInitialized(measurements);
  if (particleCount() == 0 || measurements.measurements.empty()) {
    return;
  }
  const double denom = measurementDenominator();
  auto weights = computeMeasurementWeights(measurements, denom);
  applyWeights(weights);
}

void APFParticleRepresentation::update(const MeasurementSet_t &measurements) {
  ensureInitialized(measurements);
  if (particleCount() == 0 || measurements.measurements.empty()) {
    return;
  }
  const double denom = measurementDenominator();
  auto auxWeights = computeAuxiliaryWeights(measurements, denom);
  bool hasPositive = false;
  for (double weight : auxWeights) {
    if (weight > 0.0) {
      hasPositive = true;
      break;
    }
  }
  if (hasPositive) {
    resampleWithWeights(auxWeights);
  }
  auto weights = computeMeasurementWeights(measurements, denom);
  applyWeights(weights);
}

void RPFParticleRepresentation::update(const MeasurementSet_t &measurements) {
  ensureInitialized(measurements);
  if (particleCount() == 0 || measurements.measurements.empty()) {
    return;
  }
  const double denom = measurementDenominator();
  auto weights = computeMeasurementWeights(measurements, denom);
  applyWeights(weights);
  resample();
  regularize();
}

} // namespace rfs

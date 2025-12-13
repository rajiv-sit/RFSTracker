#include "representations/ParticleRepresentation.hpp"

#include <algorithm>
#include <cmath>
#include <random>

namespace rfs {

void ParticleRepresentation::predict(double dt) {
  std::normal_distribution<double> processNoise(0.0, 0.1);
  std::mt19937 rng(std::random_device{}());

  for (auto &particle : particles_) {
    particle.state[0] += particle.state[2] * dt + processNoise(rng);
    particle.state[1] += particle.state[3] * dt + processNoise(rng);
  }
}

void ParticleRepresentation::update(const MeasurementSet_t &measurements) {
  if (measurements.measurements.empty()) {
    return;
  }

  const double sigma = 5.0;
  const double denom = 2.0 * sigma * sigma;
  double totalWeight = 0.0;

  for (auto &particle : particles_) {
    double weight = 1.0;
    for (const auto &measurement : measurements.measurements) {
      const Eigen::Vector2d statePos = particle.state.head<2>();
      const double distance = (statePos - measurement.value).norm();
      weight *= std::exp(-distance * distance / denom);
    }
    particle.weight = std::max(1e-6, weight);
    totalWeight += particle.weight;
  }

  if (totalWeight <= 0.0) {
    const double uniformWeight = 1.0 / std::max<size_t>(particles_.size(), 1);
    for (auto &particle : particles_) {
      particle.weight = uniformWeight;
    }
    totalWeight = 1.0;
  }

  for (auto &particle : particles_) {
    particle.weight /= totalWeight;
  }

  resample();
}

std::vector<Eigen::Vector4d> ParticleRepresentation::estimate() const {
  std::vector<Eigen::Vector4d> states;
  states.reserve(particles_.size());
  for (const auto &particle : particles_) {
    states.push_back(particle.state);
  }
  return states;
}

void ParticleRepresentation::setParticles(std::vector<Particle_t> particles) {
  particles_ = std::move(particles);
}

void ParticleRepresentation::resample() {
  if (particles_.empty()) {
    return;
  }

  std::vector<double> weights;
  weights.reserve(particles_.size());
  for (const auto &particle : particles_) {
    weights.push_back(particle.weight);
  }

  std::discrete_distribution<size_t> dist(weights.begin(), weights.end());
  std::mt19937 rng(std::random_device{}());
  std::vector<Particle_t> resampled;
  resampled.reserve(particles_.size());

  for (size_t i = 0; i < particles_.size(); ++i) {
    resampled.push_back(particles_[dist(rng)]);
    resampled.back().weight = 1.0 / particles_.size();
  }

  particles_ = std::move(resampled);
}

const std::vector<Particle_t> &ParticleRepresentation::particles() const {
  return particles_;
}

} // namespace rfs

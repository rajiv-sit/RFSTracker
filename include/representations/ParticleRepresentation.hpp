#pragma once

#include "representations/IRepresentation.hpp"

#include <Eigen/Dense>
#include <random>
#include <vector>

namespace rfs {

struct Particle_t {
  Eigen::Vector4d state = Eigen::Vector4d::Zero();
  double weight = 1.0;
};

class ParticleRepresentationBase : public IRepresentation {
public:
  ParticleRepresentationBase();
  ~ParticleRepresentationBase() override = default;

  void predict(double dt) override;
  void update(const MeasurementSet_t &measurements) override = 0;
  std::vector<Eigen::Vector4d> estimate() const override;

  void setParticles(std::vector<Particle_t> particles);
  const std::vector<Particle_t> &particles() const;

protected:
  void ensureInitialized(const MeasurementSet_t &measurements);

  std::vector<double> computeMeasurementWeights(const MeasurementSet_t &measurements,
                                                double denom) const;
  std::vector<double> computeAuxiliaryWeights(const MeasurementSet_t &measurements,
                                              double denom) const;
  void applyWeights(std::vector<double> &weights);
  void resample();
  void resampleWithWeights(const std::vector<double> &weights);
  void regularize(double stddev = 0.25);

  double measurementDenominator() const;

  size_t particleCount() const;

  std::vector<Particle_t> particles_;

private:
  void normalizeWeights(std::vector<double> &weights) const;
  double measurementLikelihood(const Particle_t &particle,
                               const MeasurementSet_t &measurements,
                               double denom) const;
  void spawnParticles(const Measurement_t &measurement, size_t count);
  std::vector<const Measurement_t *> collectNewDetectionMeasurements(
      const std::vector<const Measurement_t *> &detections) const;
  void recordDetectionPositions(
      const std::vector<const Measurement_t *> &detections);

  std::mt19937 rng_;
  std::vector<Eigen::Vector2d> trackedDetectionPositions_;
};

class SIRParticleRepresentation : public ParticleRepresentationBase {
public:
  void update(const MeasurementSet_t &measurements) override;
};

class SISParticleRepresentation : public ParticleRepresentationBase {
public:
  void update(const MeasurementSet_t &measurements) override;
};

class APFParticleRepresentation : public ParticleRepresentationBase {
public:
  void update(const MeasurementSet_t &measurements) override;
};

class RPFParticleRepresentation : public ParticleRepresentationBase {
public:
  void update(const MeasurementSet_t &measurements) override;
};

using ParticleRepresentation = SIRParticleRepresentation;

} // namespace rfs

#pragma once

#pragma once

#include "representations/IRepresentation.hpp"

#include <Eigen/Dense>
#include <vector>

namespace rfs {

struct Particle_t {
  Eigen::Vector4d state = Eigen::Vector4d::Zero();
  double weight = 1.0;
};

/** @brief Particle cloud representation for nonlinear dynamics. */
class ParticleRepresentation : public IRepresentation {
public:
  void predict(double dt) override;
  void update(const MeasurementSet_t &measurements) override;
  std::vector<Eigen::Vector4d> estimate() const override;

  void setParticles(std::vector<Particle_t> particles);
  void resample();
  const std::vector<Particle_t> &particles() const;

private:
  std::vector<Particle_t> particles_;
};

} // namespace rfs

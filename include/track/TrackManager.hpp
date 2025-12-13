#pragma once

#include "association/HungarianSolver.hpp"
#include "simulation/MeasurementSet.hpp"
#include "simulation/TargetState.hpp"

#include <Eigen/Dense>
#include <vector>

namespace rfs {

enum class TrackStatus { Newborn, Tentative, Confirmed };

struct TrackState {
  int id = -1;
  Eigen::Vector2d position = Eigen::Vector2d::Zero();
  Eigen::Vector2d velocity = Eigen::Vector2d::Zero();
  int hits = 0;
  int misses = 0;
  double lastUpdateTime = 0.0;
  TrackStatus status = TrackStatus::Newborn;
};

/** @brief Manages track lifecycle using assignment. */
class TrackManager {
public:
  TrackManager();

  void update(const MeasurementSet_t &measurements);
  const std::vector<TrackState> &tracks() const;

private:
  TrackStatus statusForHits(int hits) const;

  std::vector<TrackState> tracks_;
  int nextId_ = 0;
  HungarianSolver solver_;
};

} // namespace rfs

#pragma once

#include "association/HungarianSolver.hpp"
#include "simulation/MeasurementSet.hpp"

#include <Eigen/Dense>
#include <optional>
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
  std::optional<int> truthId;
};

/** @brief Manages track lifecycle using assignment. */
class TrackManager {
public:
  TrackManager();

  void update(const MeasurementSet_t &measurements);
  const std::vector<TrackState> &tracks() const;
  const std::vector<TrackState> &confirmedTracks() const;

private:
  TrackStatus statusForHits(int hits) const;
  void pruneDeadTracks();
  void refreshConfirmedTracks();

  std::vector<TrackState> tracks_;
  std::vector<TrackState> confirmedTracks_;
  int nextId_ = 1;
  HungarianSolver solver_;
};

} // namespace rfs

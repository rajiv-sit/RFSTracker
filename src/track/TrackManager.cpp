#include "track/TrackManager.hpp"

#include <algorithm>

namespace rfs {

constexpr int kMaxMissesBeforeDrop = 2;

TrackManager::TrackManager() = default;

void TrackManager::update(const MeasurementSet_t &measurements) {
  const auto &measurementList = measurements.measurements;
  if (measurementList.empty()) {
    for (auto &track : tracks_) {
      ++track.misses;
    }
    pruneDeadTracks();
    return;
  }

  if (tracks_.empty()) {
    for (const auto &measurement : measurementList) {
      TrackState track;
      track.id = nextId_++;
      track.position = measurement.value;
      track.hits = 1;
      track.lastUpdateTime = measurement.time;
      track.status = TrackStatus::Newborn;
      track.truthId = measurement.truthId;
      tracks_.push_back(track);
    }
    return;
  }

  const size_t trackCount = tracks_.size();
  const size_t measurementCount = measurementList.size();
  CostMatrix_t costMatrix(trackCount, std::vector<double>(measurementCount));

  for (size_t i = 0; i < trackCount; ++i) {
    for (size_t j = 0; j < measurementCount; ++j) {
      const double distance = (tracks_[i].position - measurementList[j].value).norm();
      costMatrix[i][j] = distance;
    }
  }

  AssociationResult_t assignment = solver_.solve(costMatrix);

  std::vector<bool> measurementAssigned(measurementCount, false);

  for (size_t i = 0; i < trackCount; ++i) {
    auto &track = tracks_[i];
    if (assignment.assignment[i] >= 0 &&
        static_cast<size_t>(assignment.assignment[i]) < measurementCount) {
      const auto &measurement = measurementList[assignment.assignment[i]];
      const Eigen::Vector2d previousPosition = track.position;
      const double previousTime = track.lastUpdateTime;
      const double deltaTime = measurement.time - previousTime;

      track.position = measurement.value;
      if (deltaTime > 1e-6) {
        track.velocity = (measurement.value - previousPosition) / deltaTime;
      } else if (track.hits == 0) {
        track.velocity = Eigen::Vector2d::Zero();
      }

      track.truthId = measurement.truthId;

      track.hits += 1;
      track.misses = 0;
      track.lastUpdateTime = measurement.time;
      track.status = statusForHits(track.hits);
      measurementAssigned[assignment.assignment[i]] = true;
    } else {
      ++track.misses;
    }
  }

  for (size_t j = 0; j < measurementCount; ++j) {
    if (!measurementAssigned[j]) {
      TrackState newTrack;
      newTrack.id = nextId_++;
      newTrack.position = measurementList[j].value;
      newTrack.hits = 1;
      newTrack.lastUpdateTime = measurementList[j].time;
      newTrack.status = TrackStatus::Newborn;
      newTrack.truthId = measurementList[j].truthId;
      tracks_.push_back(newTrack);
    }
  }

  pruneDeadTracks();
}

const std::vector<TrackState> &TrackManager::tracks() const {
  return tracks_;
}

TrackStatus TrackManager::statusForHits(int hits) const {
  if (hits <= 1) {
    return TrackStatus::Newborn;
  }
  if (hits == 2) {
    return TrackStatus::Tentative;
  }
  return TrackStatus::Confirmed;
}

void TrackManager::pruneDeadTracks() {
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                     [](const TrackState &track) { return track.misses > kMaxMissesBeforeDrop; }),
      tracks_.end());
}

} // namespace rfs

#include "track/TrackManager.hpp"

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iterator>
#include <sstream>
#include <string>

#include "logging/LoggerControl.hpp"
#include "track/TrackLogger.hpp"

namespace rfs {

constexpr int kMaxMissesBeforeDrop = 2;

namespace {
struct TrackLogWriter {
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
      stream.open("track_manager_debug.log", std::ios::trunc);
    }
  }

  std::ofstream stream;
};

TrackLogWriter &trackLogger() {
  static TrackLogWriter instance;
  return instance;
}

std::string scanPrefix() {
  std::ostringstream oss;
  oss << "[scan " << trackScanId() << "]";
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
    oss << "unknown";
  }
  return oss.str();
}
} // namespace

TrackManager::TrackManager() = default;

void TrackManager::update(const MeasurementSet_t &measurements) {
  const auto &measurementList = measurements.measurements;
  trackLogger().logLine(scanPrefix() + " [TrackManager::update] measurementCount=" +
                        std::to_string(measurementList.size()) + " trackCount=" +
                        std::to_string(tracks_.size()));
  if (measurementList.empty()) {
    for (auto &track : tracks_) {
      ++track.misses;
    }
    trackLogger().logLine(scanPrefix() + " [TrackManager::update] no measurements; incremented misses for " +
                          std::to_string(tracks_.size()) + " tracks");
    pruneDeadTracks();
    refreshConfirmedTracks();
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
    trackLogger().logLine(scanPrefix() + " [TrackManager::update] created " +
                          std::to_string(measurementList.size()) +
                          " tracks from measurements");
    refreshConfirmedTracks();
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
      {
        std::ostringstream line;
        line << scanPrefix() << " [TrackManager::update] trackId=" << track.id
             << " assigned measurement[" << assignment.assignment[i] << "] "
             << formatMeasurement(measurement);
        trackLogger().logLine(line.str());
      }
    } else {
      ++track.misses;
      trackLogger().logLine(scanPrefix() + " [TrackManager::update] trackId=" +
                            std::to_string(track.id) + " missed measurement");
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
    {
      std::ostringstream line;
      line << scanPrefix() << " [TrackManager::update] created new trackId=" << newTrack.id
           << " measurement[" << j << "] " << formatMeasurement(measurementList[j]);
      trackLogger().logLine(line.str());
    }
  }
  }

  pruneDeadTracks();
  refreshConfirmedTracks();
}

const std::vector<TrackState> &TrackManager::tracks() const {
  return tracks_;
}

const std::vector<TrackState> &TrackManager::confirmedTracks() const {
  return confirmedTracks_;
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
  std::vector<int> dropped;
  for (const auto &track : tracks_) {
    if (track.misses > kMaxMissesBeforeDrop) {
      dropped.push_back(track.id);
    }
  }
  tracks_.erase(
      std::remove_if(tracks_.begin(), tracks_.end(),
                      [](const TrackState &track) { return track.misses > kMaxMissesBeforeDrop; }),
      tracks_.end());
  if (!dropped.empty()) {
    std::ostringstream line;
    line << scanPrefix() << " [TrackManager::prune] dropped tracks:";
    for (int id : dropped) {
      line << " " << id;
    }
    trackLogger().logLine(line.str());
  }
}

void TrackManager::refreshConfirmedTracks() {
  confirmedTracks_.clear();
  std::copy_if(tracks_.begin(), tracks_.end(), std::back_inserter(confirmedTracks_),
               [](const TrackState &track) { return track.status == TrackStatus::Confirmed; });
}

} // namespace rfs

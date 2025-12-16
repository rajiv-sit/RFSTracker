#include "logging/TruthTrackLogger.hpp"

#include <fstream>
#include <iomanip>

namespace rfs {

namespace {
struct TruthTrackLogWriter {
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
      stream.open("truth_track_debug.log", std::ios::trunc);
    }
  }

  std::ofstream stream;
};

TruthTrackLogWriter &writer() {
  static TruthTrackLogWriter instance;
  return instance;
}
} // namespace

namespace {
std::string scanPrefix(int scanId) {
  std::ostringstream oss;
  oss << "[scan " << scanId << "]";
  return oss.str();
}
} // namespace

void logTruthTrackComparison(int scanId, const TrackState &track,
                             const TargetState_t &truth, double distance) {
  std::ostringstream oss;
  oss << scanPrefix(scanId) << " trackId=" << track.id << " truthId=" << truth.id
      << " distance=" << std::fixed << std::setprecision(2) << distance;
  oss << " trackPos=(" << track.position.x() << "," << track.position.y() << ")";
  oss << " truthPos=(" << truth.state.x() << "," << truth.state.y() << ")";
  writer().logLine(oss.str());
}

} // namespace rfs

#include "track/TrackLogger.hpp"

namespace rfs {

namespace {
int currentScanId = 0;
} // namespace

void setTrackScanId(int scanId) {
  currentScanId = scanId;
}

int trackScanId() {
  return currentScanId;
}

} // namespace rfs

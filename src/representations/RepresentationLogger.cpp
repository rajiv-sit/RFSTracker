#include "representations/RepresentationLogger.hpp"

namespace rfs {

namespace {
int currentScanId = 0;
} // namespace

void setRepresentationScanId(int scanId) {
  currentScanId = scanId;
}

int representationScanId() {
  return currentScanId;
}

} // namespace rfs

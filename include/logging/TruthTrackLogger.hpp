#pragma once

#include "logging/LoggerControl.hpp"
#include "track/TrackManager.hpp"
#include "simulation/TargetState.hpp"

#include <Eigen/Dense>
#include <sstream>
#include <string>

namespace rfs {

void logTruthTrackComparison(int scanId, const TrackState &track,
                             const TargetState_t &truth, double distance);

} // namespace rfs

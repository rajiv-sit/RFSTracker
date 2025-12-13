#include "simulation/GroundTruthGenerator.hpp"

#include <algorithm>
#include <cmath>
#include <iterator>

namespace rfs {

GroundTruthGenerator::GroundTruthGenerator(const TrackerConfig &config)
    : config_(config) {}

void GroundTruthGenerator::step(double currentTime, double dt) {
  activeTargets_.clear();

  for (const auto &descriptor : config_.targets) {
    if (currentTime < descriptor.startTime || currentTime > descriptor.endTime) {
      continue;
    }

    auto it = std::find_if(states_.begin(), states_.end(),
                           [&](const TargetState_t &state) { return state.id == descriptor.id; });

    if (it == states_.end()) {
      TargetState_t startState;
      startState.id = descriptor.id;
      startState.state[0] = descriptor.initialState[0];
      startState.state[1] = descriptor.initialState[1];
      startState.state[2] = descriptor.initialState[2];
      startState.state[3] = descriptor.initialState[3];
      startState.birthTime = currentTime;
      startState.deathTime = descriptor.endTime;
      states_.push_back(startState);
      it = std::prev(states_.end());
    }

    auto &state = *it;
    updateState(state, descriptor, currentTime, dt);
    activeTargets_.push_back(state);
  }
}

const std::vector<TargetState_t> &GroundTruthGenerator::activeTargets() const {
  return activeTargets_;
}

void GroundTruthGenerator::updateState(TargetState_t &state,
                                       const TargetDescriptor &descriptor,
                                       double currentTime, double dt) {
  if (currentTime - state.lastManeuverTime >= descriptor.maneuverInterval) {
    state.lastManeuverTime = currentTime;
    const double turnAngle = (state.id % 2 == 0) ? -0.3 : 0.3;
    const double speedAdjustment = 0.5;
    state.state[2] += speedAdjustment * std::cos(turnAngle);
    state.state[3] += speedAdjustment * std::sin(turnAngle);
  }

  state.state[0] += state.state[2] * dt;
  state.state[1] += state.state[3] * dt;

  const double halfWidth = config_.areaWidth * 0.5;
  const double halfHeight = config_.areaHeight * 0.5;

  if (state.state[0] > halfWidth) {
    state.state[0] = halfWidth;
    state.state[2] = -std::abs(state.state[2]);
  } else if (state.state[0] < -halfWidth) {
    state.state[0] = -halfWidth;
    state.state[2] = std::abs(state.state[2]);
  }

  if (state.state[1] > halfHeight) {
    state.state[1] = halfHeight;
    state.state[3] = -std::abs(state.state[3]);
  } else if (state.state[1] < -halfHeight) {
    state.state[1] = -halfHeight;
    state.state[3] = std::abs(state.state[3]);
  }
}

} // namespace rfs

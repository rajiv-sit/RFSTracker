#include "config/TrackerConfig.hpp"

#include <cassert>

int main() {
  auto config = rfs::TrackerConfig::defaultConfig();
  assert(config.validate());
  assert(!config.sensors.empty());
  assert(!config.targets.empty());
  return 0;
}

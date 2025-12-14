#include "logging/LoggerControl.hpp"

namespace rfs {

namespace {
bool g_loggerVerbose = true;
} // namespace

void setLoggerVerbose(bool enabled) {
  g_loggerVerbose = enabled;
}

bool loggerVerboseEnabled() {
  return g_loggerVerbose;
}

} // namespace rfs

#include "filters/FilterLogger.hpp"
#include "logging/LoggerControl.hpp"

#include <fstream>
#include <sstream>

namespace rfs {

namespace {
int currentScanId = 0;

struct FilterLogWriter {
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
      stream.open("filter_debug.log", std::ios::trunc);
    }
  }

  std::ofstream stream;
};

FilterLogWriter &filterLogger() {
  static FilterLogWriter instance;
  return instance;
}

std::string scanPrefix() {
  std::ostringstream oss;
  oss << "[scan " << currentScanId << "]";
  return oss.str();
}
} // namespace

void setFilterScanId(int scanId) {
  currentScanId = scanId;
}

int filterScanId() {
  return currentScanId;
}

void logFilterAction(const std::string &filterName, const std::string &method,
                     const std::string &details) {
  std::ostringstream oss;
  oss << scanPrefix() << " [" << filterName << "::" << method << "]";
  if (!details.empty()) {
    oss << " " << details;
  }
  filterLogger().logLine(oss.str());
}

} // namespace rfs

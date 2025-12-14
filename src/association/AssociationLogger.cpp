#include "association/AssociationLogger.hpp"
#include "logging/LoggerControl.hpp"

#include <fstream>
#include <sstream>

namespace rfs {

namespace {
int currentScanId = 0;

struct AssociationLogWriter {
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
      stream.open("association_debug.log", std::ios::trunc);
    }
  }

  std::ofstream stream;
};

AssociationLogWriter &associationLogger() {
  static AssociationLogWriter instance;
  return instance;
}

std::string scanPrefix() {
  std::ostringstream oss;
  oss << "[scan " << currentScanId << "]";
  return oss.str();
}
} // namespace

void setAssociationScanId(int scanId) {
  currentScanId = scanId;
}

int associationScanId() {
  return currentScanId;
}

void logAssociation(const std::string &message) {
  std::ostringstream oss;
  oss << scanPrefix() << " " << message;
  associationLogger().logLine(oss.str());
}

} // namespace rfs

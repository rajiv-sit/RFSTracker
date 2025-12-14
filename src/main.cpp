#include "config/TrackerConfig.hpp"
#include "logging/LoggerControl.hpp"
#include "pipeline/TrackingPipeline.hpp"
#include "visualization/ImGuiVisualizer.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

void logMessage(const std::string &message) {
  if (!rfs::loggerVerboseEnabled()) {
    return;
  }
  std::ofstream log("rfs_app_debug.log", std::ios::app);
  log << message << std::endl;
}

int main(int argc, char **argv) {
  logMessage("main: start");
  auto config = std::make_shared<rfs::TrackerConfig>(rfs::TrackerConfig::defaultConfig());

  const std::filesystem::path inputPath =
      (argc > 1) ? std::filesystem::path(argv[1])
                 : std::filesystem::path("config/default_tracker_config.json");
  std::filesystem::path resolvedPath = inputPath;
  if (!resolvedPath.is_absolute() && !std::filesystem::exists(resolvedPath)) {
    const auto exeDir = std::filesystem::path(argv[0]).parent_path();
    const auto candidate = exeDir / inputPath;
    if (std::filesystem::exists(candidate)) {
      resolvedPath = candidate;
    }
  }

  if (std::filesystem::exists(resolvedPath)) {
    if (!config->loadFromJson(resolvedPath.string())) {
      std::cerr << "WARN: Failed to parse " << resolvedPath << ", using defaults.\n";
      logMessage("main: failed to parse config");
    }
  } else {
    std::cerr << "INFO: Config file " << resolvedPath << " not found; using defaults.\n";
    logMessage("main: config file not found " + resolvedPath.string());
  }

  if (!config->validate()) {
    std::cerr << "ERROR: TrackerConfig validation failed.\n";
    logMessage("main: config invalid");
    return 1;
  }

  rfs::setLoggerVerbose(config->loggerVerbose);

  auto visualizer = std::make_unique<rfs::ImGuiVisualizer>(config);
  if (!visualizer->initialize()) {
    std::cerr << "ERROR: Visualizer initialization failed.\n";
    logMessage("main: visualizer init failed");
    return 1;
  }

  logMessage("main: visualizer initialized");
  rfs::TrackingPipeline pipeline(config, std::move(visualizer));
  logMessage("main: pipeline created");
  for (int i = 0; i < config->maxSteps && !pipeline.shouldStop(); ++i) {
    logMessage("main: step start " + std::to_string(i));
    pipeline.step(config->samplingTime);
    logMessage("main: step end " + std::to_string(i));
  }
  logMessage("main: exiting");
  return 0;
}

#include "config/TrackerConfig.hpp"
#include "pipeline/TrackingPipeline.hpp"
#include "visualization/ImGuiVisualizer.hpp"

#include <filesystem>
#include <fstream>
#include <iostream>
#include <memory>

void logMessage(const std::string &message) {
  std::ofstream log("rfs_app_debug.log", std::ios::app);
  log << message << std::endl;
}

int main(int argc, char **argv) {
  logMessage("main: start");
  auto config = std::make_shared<rfs::TrackerConfig>(rfs::TrackerConfig::defaultConfig());

  const std::string configPath =
      (argc > 1) ? std::string(argv[1]) : "config/default_tracker_config.json";

  if (std::filesystem::exists(configPath)) {
    if (!config->loadFromJson(configPath)) {
      std::cerr << "WARN: Failed to parse " << configPath << ", using defaults.\n";
      logMessage("main: failed to parse config");
    }
  } else {
    std::cerr << "INFO: Config file " << configPath << " not found; using defaults.\n";
    logMessage("main: config file not found");
  }

  if (!config->validate()) {
    std::cerr << "ERROR: TrackerConfig validation failed.\n";
    logMessage("main: config invalid");
    return 1;
  }

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

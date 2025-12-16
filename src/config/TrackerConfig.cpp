#include "config/TrackerConfig.hpp"

#include <algorithm>
#include <fstream>

namespace rfs {

using Json = nlohmann::json;

TrackerConfig TrackerConfig::defaultConfig() {
  TrackerConfig config;
  config.samplingTime = 0.1;
  config.maxSteps = 600;
  config.maxTargets = 50;
  config.areaWidth = 5000.0;
  config.areaHeight = 5000.0;
  config.clutterRate = 15.0;
  config.snrDb = 20.0;
  config.enableVisualization = true;
  config.filterFamily = FilterFamily::PHD;
  config.representation = RepresentationType::GaussianMixture;
  config.coreParticleType = ParticleFilterType::SIR;
  config.loggerVerbose = true;
  config.truthMatchThreshold = 80.0;

  config.sensors = {{
      SensorType::Radar,
      0.98,
      4.0,
      5000.0,
      0.01,
      5.0,
      20.0,
  }};

  config.targets = {{
      1,
      0.0,
      30.0,
      {1000.0, 1000.0, 10.0, 0.0},
      6.0,
  },
  {
      2,
      5.0,
      40.0,
      {2000.0, -500.0, -8.0, 2.0},
      8.0,
  },
  {
      3,
      10.0,
      55.0,
      {-1500.0, 1200.0, 12.0, -3.0},
      4.0,
  }};

  return config;
}

bool TrackerConfig::loadFromJson(const std::string &path) {
  std::ifstream input(path);
  if (!input.is_open()) {
    return false;
  }

  Json document;
  input >> document;

  if (document.contains("samplingTime")) {
    samplingTime = document["samplingTime"].get<double>();
  }
  if (document.contains("maxSteps")) {
    maxSteps = document["maxSteps"].get<int>();
  }
  if (document.contains("maxTargets")) {
    maxTargets = document["maxTargets"].get<int>();
  }
  if (document.contains("areaWidth")) {
    areaWidth = document["areaWidth"].get<double>();
  }
  if (document.contains("areaHeight")) {
    areaHeight = document["areaHeight"].get<double>();
  }
  if (document.contains("clutterRate")) {
    clutterRate = document["clutterRate"].get<double>();
  }
  if (document.contains("snrDb")) {
    snrDb = document["snrDb"].get<double>();
  }
  if (document.contains("enableVisualization")) {
    enableVisualization = document["enableVisualization"].get<bool>();
  }
  if (document.contains("truthMatchThreshold")) {
    truthMatchThreshold = document["truthMatchThreshold"].get<double>();
  }
  if (document.contains("logger_verbose")) {
    loggerVerbose = document["logger_verbose"].get<bool>();
  }
  if (document.contains("loggerVerbose")) {
    loggerVerbose = document["loggerVerbose"].get<bool>();
  }

  if (document.contains("sensors") && document["sensors"].is_array()) {
    sensors.clear();
    for (const auto &entry : document["sensors"]) {
      SensorConfig sensor;
      if (parseSensor(entry, sensor)) {
        sensors.push_back(sensor);
      }
    }
  }

  if (sensors.empty()) {
    sensors = defaultConfig().sensors;
  }

  if (document.contains("targets") && document["targets"].is_array()) {
    targets.clear();
    for (const auto &entry : document["targets"]) {
      TargetDescriptor target;
      if (parseTarget(entry, target)) {
        targets.push_back(target);
      }
    }
  }

  if (document.contains("filterFamily")) {
    filterFamily = filterFamilyFromString(document["filterFamily"].get<std::string>());
  }
  if (document.contains("representation")) {
    representation =
        representationFromString(document["representation"].get<std::string>());
  }
  if (document.contains("core_particle_type")) {
    coreParticleType =
        particleFilterTypeFromString(document["core_particle_type"].get<std::string>());
  }
  if (document.contains("coreParticleType")) {
    coreParticleType =
        particleFilterTypeFromString(document["coreParticleType"].get<std::string>());
  }

  if (targets.empty()) {
    targets = defaultConfig().targets;
  }

  return true;
}

bool TrackerConfig::validate() const {
  if (samplingTime <= 0.0 || maxSteps <= 0 || maxTargets <= 0) {
    return false;
  }
  if (areaWidth <= 0.0 || areaHeight <= 0.0) {
    return false;
  }
  if (sensors.empty()) {
    return false;
  }
  if (truthMatchThreshold <= 0.0) {
    return false;
  }
  return true;
}

SensorType TrackerConfig::sensorTypeFromString(const std::string &value) {
  if (value == "Radar") {
    return SensorType::Radar;
  }
  if (value == "Lidar") {
    return SensorType::Lidar;
  }
  if (value == "Camera") {
    return SensorType::Camera;
  }
  return SensorType::Radar;
}

bool TrackerConfig::parseSensor(const Json &entry, SensorConfig &out) {
  if (!entry.is_object()) {
    return false;
  }
  out.type = sensorTypeFromString(entry.value("type", "Radar"));
  out.detectionProbability = entry.value("detectionProbability", out.detectionProbability);
  out.falseAlarmRate = entry.value("falseAlarmRate", out.falseAlarmRate);
  out.range = entry.value("range", out.range);
  out.azimuthStdDev = entry.value("azimuthStdDev", out.azimuthStdDev);
  out.rangeStdDev = entry.value("rangeStdDev", out.rangeStdDev);
  out.signalToNoiseDb = entry.value("signalToNoiseDb", out.signalToNoiseDb);
  return true;
}

FilterFamily TrackerConfig::filterFamilyFromString(const std::string &value) {
  if (value == "CPHD") {
    return FilterFamily::CPHD;
  }
  if (value == "MB") {
    return FilterFamily::MB;
  }
  if (value == "GLMB") {
    return FilterFamily::GLMB;
  }
  return FilterFamily::PHD;
}

RepresentationType TrackerConfig::representationFromString(const std::string &value) {
  if (value == "Particles") {
    return RepresentationType::Particle;
  }
  if (value == "Spline") {
    return RepresentationType::Spline;
  }
  return RepresentationType::GaussianMixture;
}

ParticleFilterType TrackerConfig::particleFilterTypeFromString(const std::string &value) {
  if (value == "SIS") {
    return ParticleFilterType::SIS;
  }
  if (value == "SIR") {
    return ParticleFilterType::SIR;
  }
  if (value == "APF") {
    return ParticleFilterType::APF;
  }
  if (value == "RPF") {
    return ParticleFilterType::RPF;
  }
  return ParticleFilterType::SIR;
}

bool TrackerConfig::parseTarget(const Json &entry, TargetDescriptor &out) {
  if (!entry.is_object()) {
    return false;
  }
  out.id = entry.value("id", out.id);
  out.startTime = entry.value("startTime", out.startTime);
  out.endTime = entry.value("endTime", out.endTime);
  out.maneuverInterval = entry.value("maneuverInterval", out.maneuverInterval);

  if (entry.contains("initialState") && entry["initialState"].is_array()) {
    auto arr = entry["initialState"];
    for (size_t idx = 0; idx < std::min<size_t>(4, arr.size()); ++idx) {
      out.initialState[idx] = arr[idx].get<double>();
    }
  }
  return true;
}

} // namespace rfs

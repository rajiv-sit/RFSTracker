#pragma once

#include <array>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

namespace rfs {

enum class SensorType { Radar, Lidar, Camera };

struct SensorConfig {
  SensorType type = SensorType::Radar;
  double detectionProbability = 0.98;
  double falseAlarmRate = 5.0;
  double range = 5000.0;
  double azimuthStdDev = 0.01;
  double rangeStdDev = 5.0;
  double signalToNoiseDb = 20.0;
};

struct TargetDescriptor {
  int id = 0;
  double startTime = 0.0;
  double endTime = 20.0;
  std::array<double, 4> initialState = {0.0, 0.0, 0.0, 0.0};
  double maneuverInterval = 5.0;
};

enum class FilterFamily { PHD, CPHD, MB, GLMB };
enum class RepresentationType { GaussianMixture, Particle, Spline };

class TrackerConfig {
public:
  static TrackerConfig defaultConfig();

  bool loadFromJson(const std::string &path);
  bool validate() const;

  double samplingTime = 0.1;
  int maxSteps = 600;
  int maxTargets = 50;
  double areaWidth = 5000.0;
  double areaHeight = 5000.0;
  double clutterRate = 10.0;
  double snrDb = 20.0;
  bool enableVisualization = true;
  std::vector<SensorConfig> sensors;
  std::vector<TargetDescriptor> targets;
  FilterFamily filterFamily = FilterFamily::PHD;
  RepresentationType representation = RepresentationType::GaussianMixture;

private:
  static SensorType sensorTypeFromString(const std::string &value);
  static FilterFamily filterFamilyFromString(const std::string &value);
  static RepresentationType representationFromString(const std::string &value);
  static bool parseSensor(const nlohmann::json &entry, SensorConfig &out);
  static bool parseTarget(const nlohmann::json &entry, TargetDescriptor &out);
};
} // namespace rfs

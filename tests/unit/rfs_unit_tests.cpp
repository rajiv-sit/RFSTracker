#include "association/Hungarian.hpp"
#include "association/HungarianSolver.hpp"
#include "config/TrackerConfig.hpp"
#include "numerics/NumericsUtils.hpp"
#include "performance/PerformanceEvaluator.hpp"
#include "representations/GaussianRepresentation.hpp"
#include "representations/ParticleRepresentation.hpp"
#include "representations/RepresentationFactory.hpp"
#include "representations/SplineRepresentation.hpp"
#include "simulation/MeasurementSet.hpp"
#include "track/TrackManager.hpp"

#include <Eigen/Dense>
#include <cstdio>
#include <fstream>
#include <filesystem>
#include <gtest/gtest.h>
#include <unordered_set>
#include <vector>

using namespace rfs;

TEST(TrackerConfigTest, DefaultConfigValid) {
  auto config = TrackerConfig::defaultConfig();
  EXPECT_TRUE(config.validate());
  EXPECT_FALSE(config.sensors.empty());
  EXPECT_FALSE(config.targets.empty());
}

TEST(TrackerConfigTest, LoadFromJson) {
  const auto tmpPath =
      std::filesystem::temp_directory_path() / "tmp_tracker_config.json";
  const std::string path = tmpPath.string();
  std::ofstream out(path);
  out << R"(
  {
    "samplingTime": 0.2,
    "filterFamily": "MB",
    "representation": "Particles",
    "targets": [
      {
        "id": 9,
        "startTime": 1.0,
        "endTime": 5.0,
        "initialState": [0, 0, 1, 1],
        "maneuverInterval": 2.0
      }
    ]
  }
)";
  out.close();

  auto config = TrackerConfig::defaultConfig();
  EXPECT_TRUE(config.loadFromJson(path));
  EXPECT_EQ(config.filterFamily, FilterFamily::MB);
  EXPECT_EQ(config.representation, RepresentationType::Particle);
  EXPECT_EQ(config.targets.size(), 1u);
  std::remove(path.c_str());
}

TEST(TrackerConfigTest, LoadsCoreParticleType) {
  const auto tmpPath =
      std::filesystem::temp_directory_path() / "tmp_particle_type.json";
  const std::string path = tmpPath.string();
  std::ofstream out(path);
  out << R"(
  {
    "representation": "Particles",
    "core_particle_type": "APF"
  }
)";
  out.close();

  auto config = TrackerConfig::defaultConfig();
  EXPECT_TRUE(config.loadFromJson(path));
  EXPECT_EQ(config.representation, RepresentationType::Particle);
  EXPECT_EQ(config.coreParticleType, ParticleFilterType::APF);
  std::remove(path.c_str());
}

TEST(NumericsTest, LogSumExpAndNormalize) {
  std::vector<double> values = {0.0, 1.0, 2.0};
  const double le = NumericsUtils::logSumExp(values);
  EXPECT_NEAR(le, std::log(std::exp(0.0) + std::exp(1.0) + std::exp(2.0)), 1e-8);
  NumericsUtils::stableNormalize(values);
  double sum = 0.0;
  for (double v : values) {
    sum += v;
  }
  EXPECT_NEAR(sum, 1.0, 1e-8);
}

TEST(RepresentationFactoryTest, CreatesBackends) {
  EXPECT_NE(dynamic_cast<GaussianRepresentation *>(createRepresentation(RepresentationType::GaussianMixture).get()), nullptr);
  EXPECT_NE(dynamic_cast<ParticleRepresentation *>(createRepresentation(RepresentationType::Particle).get()), nullptr);
  EXPECT_NE(dynamic_cast<SplineRepresentation *>(createRepresentation(RepresentationType::Spline).get()), nullptr);
}

TEST(GaussianRepresentationTest, PruneAndMerge) {
  GaussianRepresentation representation;
  GaussianComponent_t comp;
  comp.weight = 1e-4;
  representation.addComponent(comp);
  GaussianComponent_t comp2;
  comp2.weight = 2.0;
  representation.addComponent(comp2);
  representation.prune(1e-3);
  EXPECT_EQ(representation.components().size(), 1u);
  representation.merge();
  EXPECT_EQ(representation.components().size(), 1u);
}

TEST(ParticleRepresentationTest, UpdateNormalizesAndResamples) {
  ParticleRepresentation representation;
  std::vector<Particle_t> particles;
  for (int i = 0; i < 10; ++i) {
    particles.push_back(Particle_t{Eigen::Vector4d::Constant(i), 1.0});
  }
  representation.setParticles(particles);

  MeasurementSet_t measurements;
  Measurement_t meas;
  meas.value = Eigen::Vector2d(5.0, 5.0);
  meas.time = 1.0;
  measurements.measurements.push_back(meas);

  representation.update(measurements);
  const auto states = representation.estimate();
  EXPECT_FALSE(states.empty());
}

TEST(SplineRepresentationTest, BuildsSplineFromMeasurements) {
  SplineRepresentation representation;
  MeasurementSet_t measurements;
  for (int i = 0; i < 5; ++i) {
    Measurement_t meas;
    meas.value = Eigen::Vector2d(1.0 * i, 2.0 * i);
    meas.time = static_cast<double>(i);
    measurements.measurements.push_back(meas);
  }
  representation.update(measurements);

  const auto states = representation.estimate();
  EXPECT_FALSE(states.empty());
}

TEST(HungarianSolverTest, SimpleAssignment) {
  HungarianSolver solver;
  CostMatrix_t cost = {{1.0, 2.0}, {3.0, 4.0}};
  const auto result = solver.solve(cost);
  ASSERT_EQ(result.assignment.size(), 2u);
  EXPECT_EQ(result.assignment[0], 0);
  EXPECT_EQ(result.assignment[1], 1);
  EXPECT_NEAR(result.totalCost, 5.0, 1e-6);
}

TEST(HungarianTemplateTest, SimpleMatrix) {
  Eigen::Matrix<double, 2, 2> matrix;
  matrix << 1.0, 2.0, 3.0, 4.0;
  Hungarian<double> solver(matrix);
  EXPECT_TRUE(solver.getStatus());
  const auto assignment = solver.assignmentZeroIndexed();
  ASSERT_EQ(assignment.size(), 2u);
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_NEAR(solver.getOverAllCost(), 5.0, 1e-6);
}

TEST(HungarianTemplateTest, RowExceedsColumns) {
  Eigen::Matrix<double, 3, 2> matrix;
  matrix << 1.0, 3.0, 2.0, 4.0, 10.0, 10.0;
  Hungarian<double> solver(matrix);
  EXPECT_TRUE(solver.getStatus());
  const auto assignment = solver.assignmentZeroIndexed();
  ASSERT_EQ(assignment.size(), 3u);
  EXPECT_EQ(assignment[0], 0);
  EXPECT_EQ(assignment[1], 1);
  EXPECT_GE(assignment[2], matrix.cols());
  EXPECT_NEAR(solver.getOverAllCost(), 5.0, 1e-6);
}

TEST(HungarianSolverTest, RectangularAssignment) {
  HungarianSolver solver;
  CostMatrix_t cost = {{1.0, 3.0}, {2.0, 4.0}, {10.0, 10.0}};
  const auto result = solver.solve(cost);
  ASSERT_EQ(result.assignment.size(), 3u);
  EXPECT_EQ(result.assignment[0], 0);
  EXPECT_EQ(result.assignment[1], 1);
  EXPECT_EQ(result.assignment[2], -1);
}

TEST(TrackManagerTest, CreatesTracksFromMeasurements) {
  TrackManager manager;
  MeasurementSet_t measurements;
  for (int i = 0; i < 2; ++i) {
    Measurement_t meas;
    meas.value = Eigen::Vector2d(i * 10.0, i * 5.0);
    meas.time = 0.1 * i;
    measurements.measurements.push_back(meas);
  }
  manager.update(measurements);
  EXPECT_EQ(manager.tracks().size(), 2u);
}

TEST(TrackManagerTest, DistinctIdsForNewTracks) {
  TrackManager manager;
  MeasurementSet_t measurements;
  for (int i = 0; i < 3; ++i) {
    Measurement_t measurement;
    measurement.value = Eigen::Vector2d(i * 1.1, i * -0.9);
    measurement.time = 0.1 * i;
    measurements.measurements.push_back(measurement);
  }

  manager.update(measurements);
  ASSERT_EQ(manager.tracks().size(), 3u);
  std::unordered_set<int> ids;
  for (const auto &track : manager.tracks()) {
    ids.insert(track.id);
  }
  EXPECT_EQ(ids.size(), manager.tracks().size());
}

TEST(TrackManagerTest, NewIdAfterDrop) {
  TrackManager manager;
  MeasurementSet_t measurements;
  Measurement_t measurement;
  measurement.value = Eigen::Vector2d(1.0, 2.0);
  measurement.time = 1.0;
  measurement.truthId = 42;
  measurements.measurements.push_back(measurement);

  for (int i = 0; i < 3; ++i) {
    measurement.time = 1.0 + static_cast<double>(i);
    measurements.measurements[0] = measurement;
    manager.update(measurements);
  }

  ASSERT_EQ(manager.confirmedTracks().size(), 1u);
  const int initialId = manager.confirmedTracks().front().id;
  EXPECT_GT(initialId, 0);

  for (int i = 0; i < 3; ++i) {
    manager.update(MeasurementSet_t{});
  }

  for (int i = 0; i < 3; ++i) {
    measurement.time = 5.0 + static_cast<double>(i);
    measurements.measurements[0] = measurement;
    manager.update(measurements);
  }

  ASSERT_EQ(manager.confirmedTracks().size(), 1u);
  EXPECT_NE(manager.confirmedTracks().front().id, initialId);
  EXPECT_GT(manager.confirmedTracks().front().id, initialId);
}

TEST(TrackManagerTest, KeepsStableIdWhileAlive) {
  TrackManager manager;
  MeasurementSet_t measurements;
  Measurement_t measurement;
  measurement.value = Eigen::Vector2d(3.0, -1.0);
  measurement.time = 1.0;
  measurement.truthId = 24;
  measurements.measurements.push_back(measurement);

  for (int i = 0; i < 3; ++i) {
    measurement.time = 1.0 + static_cast<double>(i);
    measurements.measurements[0] = measurement;
    manager.update(measurements);
  }

  ASSERT_EQ(manager.confirmedTracks().size(), 1u);
  const int id = manager.confirmedTracks().front().id;

  measurement.time = 5.0;
  measurements.measurements[0] = measurement;
  manager.update(measurements);

  ASSERT_EQ(manager.confirmedTracks().size(), 1u);
  EXPECT_EQ(manager.confirmedTracks().front().id, id);
}

TEST(TrackManagerTest, ConfirmedTracksRequireThreeHits) {
  TrackManager manager;
  MeasurementSet_t measurements;
  Measurement_t measurement;
  measurement.value = Eigen::Vector2d(0.0, 0.0);
  measurement.truthId = 8;
  measurement.time = 12.0;
  measurements.measurements.push_back(measurement);

  manager.update(measurements);
  EXPECT_TRUE(manager.confirmedTracks().empty());

  measurement.time = 13.0;
  measurements.measurements[0] = measurement;
  manager.update(measurements);
  EXPECT_TRUE(manager.confirmedTracks().empty());

  measurement.time = 14.0;
  measurements.measurements[0] = measurement;
  manager.update(measurements);
  ASSERT_EQ(manager.confirmedTracks().size(), 1u);
  EXPECT_EQ(manager.confirmedTracks().front().status, TrackStatus::Confirmed);
}

TEST(PerformanceEvaluatorTest, MetricsCompute) {
  PerformanceEvaluator evaluator;
  std::vector<TrackEstimate_t> estimates(2);
  std::vector<TruthTarget_t> truth(2);
  for (int i = 0; i < 2; ++i) {
    estimates[i].state = Eigen::Vector4d::Constant(i + 1);
    truth[i].state = Eigen::Vector4d::Constant(i);
  }
  EXPECT_GT(evaluator.computeNEES(estimates, truth), 0.0);
  EXPECT_GT(evaluator.computeRMSE(estimates, truth), 0.0);
  EXPECT_GT(evaluator.computeOSPA(estimates, truth), 0.0);
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

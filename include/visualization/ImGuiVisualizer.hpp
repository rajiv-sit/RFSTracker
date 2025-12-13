#pragma once

#include "config/TrackerConfig.hpp"
#include "visualization/Visualizer.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <filesystem>
#include <Eigen/Dense>
#include <memory>
#include <string>

namespace rfs {

/** @brief Console/ImGui-backed visualizer implementation. */
class ImGuiVisualizer final : public Visualizer {
public:
  explicit ImGuiVisualizer(std::shared_ptr<TrackerConfig> config);

  bool initialize() override;
  void renderFrame(const MeasurementSet_t &measurements,
                   const std::vector<TrackState> &tracks,
                   const std::vector<TargetState_t> &truth,
                   const PerformanceMetrics &metrics,
                   int scanId,
                   double timeElapsed) override;
  void shutdown() override;

  const VisualizerOptions &options() const override;
  void setOptions(const VisualizerOptions &options) override;
  bool shouldClose() const override;

private:
  bool createShaderProgram();
  void setupGridGeometry();
  void setupPointGeometry();
  void renderGrid() const;
  void renderPoints(const std::vector<Eigen::Vector2d> &points,
                    const Eigen::Vector3d &color,
                    float size) const;
  void showControlPanels(const MeasurementSet_t &measurements,
                         const std::vector<TrackState> &tracks,
                         const std::vector<TargetState_t> &truth,
                         const PerformanceMetrics &metrics,
                         int scanId,
                         double timeElapsed);

  std::shared_ptr<TrackerConfig> config_;
  mutable VisualizerOptions options_;

  GLFWwindow *window_ = nullptr;
  GLuint gridProgram_ = 0;
  GLuint gridVAO_ = 0;
  GLuint gridVBO_ = 0;
  int gridVertexCount_ = 0;
  GLint mvpLocation_ = -1;
  GLint colorLocation_ = -1;
  bool shouldClose_ = false;

  GLuint pointVAO_ = 0;
  GLuint pointVBO_ = 0;

  std::vector<float> rmseHistory_;
  std::vector<float> neesHistory_;
  std::vector<float> ospaHistory_;
  float areaHalfWidth_ = 0.0f;
  float areaHalfHeight_ = 0.0f;
  std::filesystem::path shaderDirectory_;
};

} // namespace rfs

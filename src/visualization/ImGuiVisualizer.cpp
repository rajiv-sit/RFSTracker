#include "visualization/ImGuiVisualizer.hpp"

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

#include <array>
#include <cfloat>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <unordered_map>
#include <limits>
#include <unordered_set>
#include <deque>
#include <vector>
#include <algorithm>
#include <Eigen/Dense>

#ifndef RFSTRACKER_SHADER_DIR
#define RFSTRACKER_SHADER_DIR ""
#endif

namespace rfs {

namespace {
GLuint compileShader(GLenum type, const char *source) {
  const GLuint id = glCreateShader(type);
  glShaderSource(id, 1, &source, nullptr);
  glCompileShader(id);

  GLint success = 0;
  glGetShaderiv(id, GL_COMPILE_STATUS, &success);
  if (success == GL_FALSE) {
    GLchar log[1024];
    glGetShaderInfoLog(id, sizeof(log), nullptr, log);
    std::cerr << "Shader compile failed: " << log << "\n";
    glDeleteShader(id);
    return 0;
  }
  return id;
}

GLuint createProgram(const char *vertexSrc, const char *fragmentSrc) {
  GLuint program = glCreateProgram();
  const GLuint vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
  const GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
  if (!vs || !fs) {
    glDeleteProgram(program);
    return 0;
  }
  glAttachShader(program, vs);
  glAttachShader(program, fs);
  glLinkProgram(program);

  GLint success = 0;
  glGetProgramiv(program, GL_LINK_STATUS, &success);
  if (success == GL_FALSE) {
    GLchar log[1024];
    glGetProgramInfoLog(program, sizeof(log), nullptr, log);
    std::cerr << "Program link failed: " << log << "\n";
    glDeleteProgram(program);
    program = 0;
  }

  glDeleteShader(vs);
  glDeleteShader(fs);
  return program;
}

std::array<float, 16> orthographic(float left,
                                   float right,
                                   float bottom,
                                   float top,
                                   float nearVal,
                                   float farVal) {
  std::array<float, 16> m{};
  m[0] = 2.0f / (right - left);
  m[5] = 2.0f / (top - bottom);
  m[10] = -2.0f / (farVal - nearVal);
  m[12] = -(right + left) / (right - left);
  m[13] = -(top + bottom) / (top - bottom);
  m[14] = -(farVal + nearVal) / (farVal - nearVal);
  m[15] = 1.0f;
  return m;
}

std::string loadShaderSource(const std::filesystem::path &path) {
  std::ifstream file(path, std::ios::binary);
  if (!file) {
    return {};
  }
  return std::string(std::istreambuf_iterator<char>(file),
                     std::istreambuf_iterator<char>());
}

const char *filterFamilyLabel(FilterFamily family) {
  switch (family) {
  case FilterFamily::CPHD:
    return "CPHD";
  case FilterFamily::MB:
    return "MB";
  case FilterFamily::GLMB:
    return "GLMB";
  case FilterFamily::PHD:
  default:
    return "PHD";
  }
}

const char *representationLabel(RepresentationType type) {
  switch (type) {
  case RepresentationType::Particle:
    return "Particles";
  case RepresentationType::Spline:
    return "Spline";
  case RepresentationType::GaussianMixture:
  default:
    return "GaussianMixture";
  }
}

const char *particleFilterTypeLabel(ParticleFilterType type) {
  switch (type) {
  case ParticleFilterType::SIS:
    return "SIS";
  case ParticleFilterType::SIR:
    return "SIR";
  case ParticleFilterType::APF:
    return "APF";
  case ParticleFilterType::RPF:
    return "RPF";
  default:
    return "Unknown";
  }
}
} // namespace

ImGuiVisualizer::ImGuiVisualizer(std::shared_ptr<TrackerConfig> config)
    : config_(std::move(config)),
      shaderDirectory_(RFSTRACKER_SHADER_DIR) {
  areaHalfWidth_ = static_cast<float>(config_->areaWidth * 0.5);
  areaHalfHeight_ = static_cast<float>(config_->areaHeight * 0.5);
  rmseHistory_.reserve(512);
  neesHistory_.reserve(512);
  ospaHistory_.reserve(512);
}

bool ImGuiVisualizer::initialize() {
  if (!glfwInit()) {
    std::cerr << "Failed to initialize GLFW\n";
    return false;
  }
  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
  glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

  window_ = glfwCreateWindow(1280, 720, "RFSTracker Visualizer", nullptr, nullptr);
  if (!window_) {
    std::cerr << "Failed to create GLFW window\n";
    glfwTerminate();
    return false;
  }
  glfwMakeContextCurrent(window_);
  glfwSwapInterval(1);

  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize OpenGL loader\n";
    return false;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGuiIO &io = ImGui::GetIO();
  io.ConfigFlags |= ImGuiConfigFlags_DockingEnable;
  io.IniFilename = "imgui.ini";

  ImGui::StyleColorsDark();
  ImGui_ImplGlfw_InitForOpenGL(window_, true);
  ImGui_ImplOpenGL3_Init("#version 330 core");

  return true;
}

bool ImGuiVisualizer::createShaderProgram() {
  const auto baseDir =
      shaderDirectory_.empty() ? (std::filesystem::current_path() / "shaders")
                               : shaderDirectory_;
  const auto vertexPath = baseDir / "grid.vs";
  const auto fragmentPath = baseDir / "grid.fs";

  const auto vertexSource = loadShaderSource(vertexPath);
  const auto fragmentSource = loadShaderSource(fragmentPath);
  if (vertexSource.empty() || fragmentSource.empty()) {
    std::cerr << "Failed to load shader sources from " << vertexPath << " and "
              << fragmentPath << "\n";
    return false;
  }

  gridProgram_ = createProgram(vertexSource.c_str(), fragmentSource.c_str());
  if (!gridProgram_) {
    return false;
  }
  mvpLocation_ = glGetUniformLocation(gridProgram_, "uMVP");
  colorLocation_ = glGetUniformLocation(gridProgram_, "uColor");
  return true;
}

void ImGuiVisualizer::setupGridGeometry() {
  std::vector<float> vertices;
  const int lines = 32;
  const float width = config_->areaWidth;
  const float height = config_->areaHeight;
  const float dx = width / static_cast<float>(lines);
  const float dy = height / static_cast<float>(lines);

  for (int i = 0; i <= lines; ++i) {
    float x = -areaHalfWidth_ + dx * i;
    vertices.push_back(x);
    vertices.push_back(-areaHalfHeight_);
    vertices.push_back(x);
    vertices.push_back(areaHalfHeight_);
  }
  for (int i = 0; i <= lines; ++i) {
    float y = -areaHalfHeight_ + dy * i;
    vertices.push_back(-areaHalfWidth_);
    vertices.push_back(y);
    vertices.push_back(areaHalfWidth_);
    vertices.push_back(y);
  }

  gridVertexCount_ = static_cast<int>(vertices.size() / 2);
  glGenVertexArrays(1, &gridVAO_);
  glGenBuffers(1, &gridVBO_);
  glBindVertexArray(gridVAO_);
  glBindBuffer(GL_ARRAY_BUFFER, gridVBO_);
  glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_STATIC_DRAW);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), reinterpret_cast<void *>(0));
  glBindVertexArray(0);

  setupPointGeometry();
}

void ImGuiVisualizer::setupPointGeometry() {
  glGenVertexArrays(1, &pointVAO_);
  glGenBuffers(1, &pointVBO_);
  glBindVertexArray(pointVAO_);
  glBindBuffer(GL_ARRAY_BUFFER, pointVBO_);
  glEnableVertexAttribArray(0);
  glVertexAttribPointer(0, 2, GL_FLOAT, GL_FALSE, 2 * sizeof(float), reinterpret_cast<void *>(0));
  glBindVertexArray(0);
}

void ImGuiVisualizer::renderFrame(const MeasurementSet_t &measurements,
                                  const std::vector<TrackState> &tracks,
                                  const std::vector<TargetState_t> &truth,
                                  const PerformanceMetrics &metrics,
                                  int scanId,
                                  double timeElapsed) {
  if (!window_) {
    return;
  }

  glfwPollEvents();
  shouldClose_ = glfwWindowShouldClose(window_);
  if (shouldClose_) {
    return;
  }

  if (metrics.truthAvailable) {
    rmseHistory_.push_back(static_cast<float>(metrics.rmse));
    neesHistory_.push_back(static_cast<float>(metrics.nees));
    ospaHistory_.push_back(static_cast<float>(metrics.ospa));
    if (rmseHistory_.size() > 256) {
      rmseHistory_.erase(rmseHistory_.begin());
      neesHistory_.erase(neesHistory_.begin());
      ospaHistory_.erase(ospaHistory_.begin());
    }
  } else {
    trackSpreadHistory_.push_back(static_cast<float>(metrics.trackSpread));
    if (trackSpreadHistory_.size() > 256) {
      trackSpreadHistory_.erase(trackSpreadHistory_.begin());
    }
  }

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();

  // Fullscreen dockspace so all panels can be docked/undocked and layout persists in imgui.ini.
  if (ImGui::GetIO().ConfigFlags & ImGuiConfigFlags_DockingEnable) {
    ImGuiViewport *viewport = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos(viewport->WorkPos);
    ImGui::SetNextWindowSize(viewport->WorkSize);
    ImGui::SetNextWindowViewport(viewport->ID);

    ImGuiWindowFlags dockspaceFlags = ImGuiWindowFlags_NoDocking | ImGuiWindowFlags_NoTitleBar |
                                      ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize |
                                      ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoBringToFrontOnFocus |
                                      ImGuiWindowFlags_NoNavFocus | ImGuiWindowFlags_NoBackground;
    ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));
    ImGui::Begin("DockSpaceHost", nullptr, dockspaceFlags);
    ImGuiID dockspaceId = ImGui::GetID("RfsDockSpace");
    ImGui::DockSpace(dockspaceId, ImVec2(0.0f, 0.0f), ImGuiDockNodeFlags_PassthruCentralNode);
    ImGui::End();
    ImGui::PopStyleVar(2);
  }

  showControlPanels(measurements, tracks, truth, scanId, timeElapsed);
  showMetricsWindow(measurements, tracks, truth, metrics);
  showTruthPanel(truth);
  showTrackTruthComparison(tracks, truth);

  updateTrails(tracks, truth);

  ImGui::SetNextWindowSize(ImVec2(600, 500), ImGuiCond_FirstUseEver);
  ImGui::Begin("World View", nullptr,
               ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse);
  ImVec2 canvasPos = ImGui::GetCursorScreenPos();
  ImVec2 canvasSize = ImGui::GetContentRegionAvail();
  if (canvasSize.x < 50.0f) {
    canvasSize.x = 50.0f;
  }
  if (canvasSize.y < 50.0f) {
    canvasSize.y = 50.0f;
  }
  const ImVec2 canvasEnd(canvasPos.x + canvasSize.x, canvasPos.y + canvasSize.y);
  ImDrawList *drawList = ImGui::GetWindowDrawList();
  drawList->AddRectFilled(canvasPos, canvasEnd, IM_COL32(15, 15, 18, 255));
  drawList->AddRect(canvasPos, canvasEnd, IM_COL32(60, 60, 70, 255));

  const auto toCanvas = [&](const Eigen::Vector2d &point) {
    const float nx =
        static_cast<float>((point.x() + areaHalfWidth_) / (2.0f * areaHalfWidth_));
    const float ny =
        static_cast<float>((point.y() + areaHalfHeight_) / (2.0f * areaHalfHeight_));
    return ImVec2(canvasPos.x + nx * canvasSize.x,
                  canvasPos.y + (1.0f - ny) * canvasSize.y);
  };
  const auto toColor = [](const Eigen::Vector3d &color) {
    return IM_COL32(static_cast<int>(color.x() * 255.0f),
                    static_cast<int>(color.y() * 255.0f),
                    static_cast<int>(color.z() * 255.0f), 255);
  };

  const int gridLines = 10;
  for (int i = 0; i <= gridLines; ++i) {
    const float t = static_cast<float>(i) / static_cast<float>(gridLines);
    const float x = canvasPos.x + t * canvasSize.x;
    const float y = canvasPos.y + t * canvasSize.y;
    drawList->AddLine(ImVec2(x, canvasPos.y), ImVec2(x, canvasEnd.y),
                      IM_COL32(40, 40, 50, 255));
    drawList->AddLine(ImVec2(canvasPos.x, y), ImVec2(canvasEnd.x, y),
                      IM_COL32(40, 40, 50, 255));
  }

  if (options_.showTruth) {
    for (const auto &entry : truthTrails_) {
      if (entry.second.size() < 2) {
        continue;
      }
      std::vector<ImVec2> points;
      points.reserve(entry.second.size());
      for (const auto &p : entry.second) {
        points.push_back(toCanvas(p));
      }
      drawList->AddPolyline(points.data(), static_cast<int>(points.size()),
                            IM_COL32(46, 217, 84, 255), ImDrawFlags_None, 2.2f);
    }
  }

  for (const auto &entry : trackTrails_) {
    if (entry.second.size() < 2) {
      continue;
    }
    std::vector<ImVec2> points;
    points.reserve(entry.second.size());
    for (const auto &p : entry.second) {
      points.push_back(toCanvas(p));
    }
    const ImU32 color = toColor(colorVectorForStatus(TrackStatus::Confirmed));
    drawList->AddPolyline(points.data(), static_cast<int>(points.size()), color,
                          ImDrawFlags_None, 2.4f);
  }

  if (options_.showMeasurements) {
    for (const auto &measurement : measurements.measurements) {
      const ImVec2 pos = toCanvas(measurement.value);
      drawList->AddCircleFilled(pos, 4.5f, IM_COL32(255, 90, 90, 255));
    }
  }

  if (options_.showTruth) {
    for (const auto &truthTarget : truth) {
      const ImVec2 pos = toCanvas(truthTarget.state.head<2>());
      drawList->AddCircleFilled(pos, 6.0f, IM_COL32(46, 217, 84, 255));
    }
  }

  if (options_.showTracks) {
    for (const auto &track : tracks) {
      if (track.status != TrackStatus::Confirmed || !isNonStationaryTrack(track)) {
        continue;
      }
      const ImVec2 pos = toCanvas(track.position);
      const ImU32 color = toColor(colorVectorForStatus(track.status));
      drawList->AddCircleFilled(pos, 7.0f, color);
      const std::string label = std::to_string(track.id);
      drawList->AddText(ImVec2(pos.x + 6.0f, pos.y - 8.0f), color, label.c_str());
    }
  }

  ImGui::Dummy(canvasSize);
  ImGui::End();

  ImGui::Render();
  int displayW, displayH;
  glfwGetFramebufferSize(window_, &displayW, &displayH);
  glViewport(0, 0, displayW, displayH);
  glClearColor(0.06f, 0.06f, 0.07f, 1.0f);
  glClear(GL_COLOR_BUFFER_BIT);

  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());
  glfwSwapBuffers(window_);
}

void ImGuiVisualizer::showControlPanels(const MeasurementSet_t &measurements,
                                        const std::vector<TrackState> &tracks,
                                        const std::vector<TargetState_t> &truth,
                                        int scanId,
                                        double timeElapsed) {
  ImGui::Begin("Tracker Controls");
  ImGui::Text("Scan #%d \u2013 Time %.2fs", scanId, timeElapsed);
  ImGui::Separator();
  ImGui::Text("Truth targets: %zu", truth.size());
  ImGui::Text("Measurements: %zu", measurements.measurements.size());
  size_t nonStationaryConfirmed = 0;
  for (const auto &track : tracks) {
    if (track.status == TrackStatus::Confirmed && isNonStationaryTrack(track)) {
      ++nonStationaryConfirmed;
    }
  }
  ImGui::Text("Non-stationary confirmed tracks: %zu", nonStationaryConfirmed);
  ImGui::Text("Filter family: %s", filterFamilyLabel(config_->filterFamily));
  ImGui::Text("Representation: %s", representationLabel(config_->representation));
  if (config_->representation == RepresentationType::Particle) {
    ImGui::Text("Core particle type: %s",
                particleFilterTypeLabel(config_->coreParticleType));
  }
  ImGui::Spacing();
  ImGui::Text("Display");
  ImGui::Checkbox("Show Truth", &options_.showTruth);
  ImGui::Checkbox("Show Measurements", &options_.showMeasurements);
  ImGui::Checkbox("Show Tracks", &options_.showTracks);
  ImGui::Checkbox("Show Truth Panel", &options_.showTruthDetails);
  if (ImGui::Button(options_.showTrackDetails ? "Hide Track Details" : "Show Track Details")) {
    options_.showTrackDetails = !options_.showTrackDetails;
  }
  if (ImGui::Button(options_.showTrackTruthComparison ? "Hide Track / Truth Comparison"
                                                     : "Show Track / Truth Comparison")) {
    options_.showTrackTruthComparison = !options_.showTrackTruthComparison;
  }
  ImGui::End();
}

void ImGuiVisualizer::showMetricsWindow(const MeasurementSet_t &measurements,
                                        const std::vector<TrackState> &tracks,
                                        const std::vector<TargetState_t> &truth,
                                        const PerformanceMetrics &metrics) {
  ImGui::Begin("Performance Metrics & Track Details");
  ImGui::Text("Measurements: %zu", measurements.measurements.size());
  ImGui::Text("Truth targets: %zu", truth.size());
  ImGui::Text("Tracks: %zu", tracks.size());
  ImGui::Separator();
  if (metrics.truthAvailable) {
    ImGui::Text("RMSE: %.2f", metrics.rmse);
    ImGui::Text("NEES: %.2f", metrics.nees);
    ImGui::Text("OSPA: %.2f", metrics.ospa);
    if (!rmseHistory_.empty()) {
      ImGui::PlotLines("RMSE", rmseHistory_.data(), static_cast<int>(rmseHistory_.size()), 0,
                       nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 60));
    }
    if (!neesHistory_.empty()) {
      ImGui::PlotLines("NEES", neesHistory_.data(), static_cast<int>(neesHistory_.size()), 0,
                       nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 60));
    }
    if (!ospaHistory_.empty()) {
      ImGui::PlotLines("OSPA", ospaHistory_.data(), static_cast<int>(ospaHistory_.size()), 0,
                       nullptr, FLT_MAX, FLT_MAX, ImVec2(0, 60));
    }
  } else {
    ImGui::TextColored(ImVec4(1.0f, 0.6f, 0.0f, 1.0f), "Truth data unavailable!");
    ImGui::Text("Track spread (avg. pairwise distance): %.2f m", metrics.trackSpread);
    if (!trackSpreadHistory_.empty()) {
      ImGui::PlotLines("Track Spread", trackSpreadHistory_.data(),
                       static_cast<int>(trackSpreadHistory_.size()), 0, nullptr,
                       FLT_MAX, FLT_MAX, ImVec2(0, 60));
    }
  }
  ImGui::Spacing();
  if (options_.showTrackDetails) {
    ImGui::Text("Track Details");
    ImGui::Columns(6, nullptr, false);
    ImGui::Text("ID"); ImGui::NextColumn();
    ImGui::Text("Status"); ImGui::NextColumn();
    ImGui::Text("Pos"); ImGui::NextColumn();
    ImGui::Text("Vel"); ImGui::NextColumn();
    ImGui::Text("Hits"); ImGui::NextColumn();
    ImGui::Text("Misses"); ImGui::NextColumn();
    ImGui::Separator();

    for (const auto &track : tracks) {
      if (track.status != TrackStatus::Confirmed || !isNonStationaryTrack(track)) {
        continue;
      }
      ImGui::Text("%d", track.id);
      ImGui::NextColumn();
      const char *statusLabel = track.status == TrackStatus::Confirmed
                                    ? "Confirmed"
                                    : (track.status == TrackStatus::Tentative ? "Tentative" : "Newborn");
      ImGui::Text("%s", statusLabel);
      ImGui::NextColumn();
      ImGui::Text("%.1f, %.1f", track.position.x(), track.position.y());
      ImGui::NextColumn();
      ImGui::Text("%.1f, %.1f", track.velocity.x(), track.velocity.y());
      ImGui::NextColumn();
      ImGui::Text("%d", track.hits);
      ImGui::NextColumn();
      ImGui::Text("%d", track.misses);
      ImGui::NextColumn();
    }

    ImGui::Columns(1);
  }
  if (options_.showTruthDetails) {
    ImGui::Text("Truth Targets");
    ImGui::Columns(3, nullptr, false);
    ImGui::Text("ID");
    ImGui::NextColumn();
    ImGui::Text("Position");
    ImGui::NextColumn();
    ImGui::Text("Velocity");
    ImGui::NextColumn();
    ImGui::Separator();
    for (const auto &target : truth) {
      ImGui::Text("%d", target.id);
      ImGui::NextColumn();
      ImGui::Text("%.1f, %.1f", target.state.x(), target.state.y());
      ImGui::NextColumn();
      ImGui::Text("%.1f, %.1f", target.state.z(), target.state.w());
      ImGui::NextColumn();
    }
    ImGui::Columns(1);
  }
  ImGui::End();
}

void ImGuiVisualizer::showTruthPanel(const std::vector<TargetState_t> &truth) {
  if (!options_.showTruthDetails) {
    return;
  }

  ImGui::Begin("Truth Details");
  ImGui::Text("Truth targets: %zu", truth.size());
  ImGui::Separator();
  ImGui::Columns(3, nullptr, false);
  ImGui::Text("ID");
  ImGui::NextColumn();
  ImGui::Text("Position");
  ImGui::NextColumn();
  ImGui::Text("Velocity");
  ImGui::NextColumn();
  ImGui::Separator();

  for (const auto &target : truth) {
    ImGui::Text("%d", target.id);
    ImGui::NextColumn();
    ImGui::Text("%.1f, %.1f", target.state.x(), target.state.y());
    ImGui::NextColumn();
    ImGui::Text("%.1f, %.1f", target.state.z(), target.state.w());
    ImGui::NextColumn();
  }

  ImGui::Columns(1);
  ImGui::End();
}

void ImGuiVisualizer::showTrackTruthComparison(const std::vector<TrackState> &tracks,
                                               const std::vector<TargetState_t> &truth) {
  if (!options_.showTrackTruthComparison) {
    return;
  }

  ImGui::Begin("Track / Truth Comparison");
  ImGui::Text("Confirmed tracks: %zu", tracks.size());
  ImGui::Text("Truth targets: %zu", truth.size());
  ImGui::Separator();
  ImGui::Columns(7, nullptr, false);
  ImGui::Text("Track ID"); ImGui::NextColumn();
  ImGui::Text("Track Pos"); ImGui::NextColumn();
  ImGui::Text("Track Vel"); ImGui::NextColumn();
  ImGui::Text("Truth ID"); ImGui::NextColumn();
  ImGui::Text("Truth Pos"); ImGui::NextColumn();
  ImGui::Text("Truth Vel"); ImGui::NextColumn();
  ImGui::Text("Distance"); ImGui::NextColumn();
  ImGui::Separator();

  std::unordered_map<int, TargetState_t> truthById;
  for (const auto &target : truth) {
    truthById[target.id] = target;
  }

  for (const auto &track : tracks) {
    if (track.status != TrackStatus::Confirmed || !isNonStationaryTrack(track)) {
      continue;
    }
    const bool hasTruth = track.truthId && truthById.contains(*track.truthId);
    Eigen::Vector2d truthPos = Eigen::Vector2d::Zero();
    Eigen::Vector2d truthVel = Eigen::Vector2d::Zero();
    double distance = std::numeric_limits<double>::quiet_NaN();
    if (hasTruth) {
      const auto &truthState = truthById[*track.truthId].state;
      truthPos = truthState.head<2>();
      truthVel = truthState.tail<2>();
      distance = (track.position - truthPos).norm();
    } else if (!truth.empty()) {
      Eigen::Vector2d closestPos = truth.front().state.head<2>();
      Eigen::Vector2d closestVel = truth.front().state.tail<2>();
      double bestDist = (track.position - closestPos).norm();
      for (const auto &target : truth) {
        const double candidateDist = (track.position - target.state.head<2>()).norm();
        if (candidateDist < bestDist) {
          bestDist = candidateDist;
          closestPos = target.state.head<2>();
          closestVel = target.state.tail<2>();
        }
      }
      truthPos = closestPos;
      truthVel = closestVel;
      distance = bestDist;
    }

    ImGui::Text("%d", track.id);
    ImGui::NextColumn();
    ImGui::Text("%.1f, %.1f", track.position.x(), track.position.y());
    ImGui::NextColumn();
    ImGui::Text("%.1f, %.1f", track.velocity.x(), track.velocity.y());
    ImGui::NextColumn();
    if (hasTruth) {
      ImGui::Text("%d", *track.truthId);
    } else {
      ImGui::Text("N/A");
    }
    ImGui::NextColumn();
    if (hasTruth) {
      ImGui::Text("%.1f, %.1f", truthPos.x(), truthPos.y());
    } else if (!truth.empty()) {
      ImGui::Text("%.1f, %.1f", truthPos.x(), truthPos.y());
    } else {
      ImGui::Text("N/A");
    }
    ImGui::NextColumn();
    if (hasTruth) {
      ImGui::Text("%.1f, %.1f", truthVel.x(), truthVel.y());
    } else if (!truth.empty()) {
      ImGui::Text("%.1f, %.1f", truthVel.x(), truthVel.y());
    } else {
      ImGui::Text("N/A");
    }
    ImGui::NextColumn();
    if (std::isnan(distance)) {
      ImGui::Text("N/A");
    } else {
      ImGui::Text("%.2f", distance);
    }
    ImGui::NextColumn();
  }

  ImGui::Columns(1);
  ImGui::End();
}

void ImGuiVisualizer::renderGrid() const {
  if (!gridProgram_ || !gridVAO_) {
    return;
  }
  glUseProgram(gridProgram_);
  const auto mvp = orthographic(-areaHalfWidth_,
                                areaHalfWidth_,
                                -areaHalfHeight_,
                                areaHalfHeight_,
                                -1.0f,
                                1.0f);
  glUniformMatrix4fv(mvpLocation_, 1, GL_FALSE, mvp.data());
  glUniform3f(colorLocation_, 0.25f, 0.27f, 0.35f);
  glBindVertexArray(gridVAO_);
  glDrawArrays(GL_LINES, 0, gridVertexCount_);
  glBindVertexArray(0);
  glUseProgram(0);
}

void ImGuiVisualizer::renderAxisLabels(int displayW, int displayH) const {
  ImGuiWindowFlags windowFlags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoInputs |
      ImGuiWindowFlags_NoBackground | ImGuiWindowFlags_AlwaysAutoResize |
      ImGuiWindowFlags_NoBringToFrontOnFocus;

  ImGui::SetNextWindowPos(ImVec2(10.0f, static_cast<float>(displayH) - 28.0f), ImGuiCond_Always);
  ImGui::Begin("##AxisXLabels", nullptr, windowFlags);
  ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 0.9f), "X (m)");
  ImGui::End();

  ImGui::SetNextWindowPos(ImVec2(10.0f, 10.0f), ImGuiCond_Always);
  ImGui::Begin("##AxisYLabels", nullptr, windowFlags);
  ImGui::TextColored(ImVec4(0.8f, 0.8f, 0.8f, 0.9f), "Y (m)");
  ImGui::End();
}

ImVec2 ImGuiVisualizer::worldToScreen(const Eigen::Vector2d &position,
                                      int displayW,
                                      int displayH) const {
  const float normalizedX =
      (static_cast<float>(position.x()) + areaHalfWidth_) / (2.0f * areaHalfWidth_);
  const float normalizedY =
      (static_cast<float>(position.y()) + areaHalfHeight_) / (2.0f * areaHalfHeight_);
  const float screenX = normalizedX * static_cast<float>(displayW);
  const float screenY = (1.0f - normalizedY) * static_cast<float>(displayH);
  return ImVec2(screenX, screenY);
}

Eigen::Vector3d ImGuiVisualizer::colorVectorForStatus(TrackStatus status) const {
  switch (status) {
  case TrackStatus::Newborn:
    return {1.0, 0.9, 0.5};
  case TrackStatus::Tentative:
    return {1.0, 0.5, 0.7};
  default:
    return {0.18, 0.6, 1.0};
  }
}

bool ImGuiVisualizer::isNonStationaryTrack(const TrackState &track) const {
  return track.velocity.norm() > kStationaryVelocityThreshold;
}

ImU32 ImGuiVisualizer::colorForStatus(TrackStatus status) const {
  const auto color = colorVectorForStatus(status);
  return IM_COL32(static_cast<int>(color.x() * 255.0f),
                  static_cast<int>(color.y() * 255.0f),
                  static_cast<int>(color.z() * 255.0f),
                  255);
}

void ImGuiVisualizer::renderPoints(const std::vector<Eigen::Vector2d> &points,
                                   const Eigen::Vector3d &color,
                                   float size) const {
  if (points.empty() || !pointVAO_ || !pointVBO_) {
    return;
  }

  const auto mvp = orthographic(-areaHalfWidth_, areaHalfWidth_, -areaHalfHeight_, areaHalfHeight_, -1.0f, 1.0f);
  glUseProgram(gridProgram_);
  glUniformMatrix4fv(mvpLocation_, 1, GL_FALSE, mvp.data());
  glUniform3f(colorLocation_, static_cast<float>(color.x()), static_cast<float>(color.y()),
              static_cast<float>(color.z()));
  glBindVertexArray(pointVAO_);
  glBindBuffer(GL_ARRAY_BUFFER, pointVBO_);

  std::vector<float> buffer;
  buffer.reserve(points.size() * 2);
  for (const auto &point : points) {
    buffer.push_back(static_cast<float>(point.x()));
    buffer.push_back(static_cast<float>(point.y()));
  }
  glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(float), buffer.data(), GL_STREAM_DRAW);

  glPointSize(size);
  glDrawArrays(GL_POINTS, 0, static_cast<GLsizei>(points.size()));
  glBindVertexArray(0);
  glPointSize(1.0f);
}

void ImGuiVisualizer::renderTrajectory(const std::vector<Eigen::Vector2d> &points,
                                       const Eigen::Vector3d &color,
                                       float thickness) const {
  if (points.size() < 2 || !pointVAO_ || !pointVBO_) {
    return;
  }

  const auto mvp = orthographic(-areaHalfWidth_, areaHalfWidth_, -areaHalfHeight_, areaHalfHeight_, -1.0f, 1.0f);
  glUseProgram(gridProgram_);
  glUniformMatrix4fv(mvpLocation_, 1, GL_FALSE, mvp.data());
  glUniform3f(colorLocation_, static_cast<float>(color.x()), static_cast<float>(color.y()),
              static_cast<float>(color.z()));
  glBindVertexArray(pointVAO_);
  glBindBuffer(GL_ARRAY_BUFFER, pointVBO_);

  std::vector<float> buffer;
  buffer.reserve(points.size() * 2);
  for (const auto &point : points) {
    buffer.push_back(static_cast<float>(point.x()));
    buffer.push_back(static_cast<float>(point.y()));
  }
  glBufferData(GL_ARRAY_BUFFER, buffer.size() * sizeof(float), buffer.data(), GL_STREAM_DRAW);

  glLineWidth(thickness);
  glDrawArrays(GL_LINE_STRIP, 0, static_cast<GLsizei>(points.size()));
  glBindVertexArray(0);
  glLineWidth(1.0f);
}

void ImGuiVisualizer::updateTrails(const std::vector<TrackState> &tracks,
                                   const std::vector<TargetState_t> &truth) {
  std::unordered_set<int> activeTrackIds;
  for (const auto &track : tracks) {
    if (track.status != TrackStatus::Confirmed || !isNonStationaryTrack(track)) {
      continue;
    }
    auto &trail = trackTrails_[track.id];
    trail.emplace_back(track.position);
    if (trail.size() > kTrailLength) {
      trail.pop_front();
    }
    activeTrackIds.insert(track.id);
  }
  for (auto it = trackTrails_.begin(); it != trackTrails_.end();) {
    if (activeTrackIds.find(it->first) == activeTrackIds.end()) {
      it = trackTrails_.erase(it);
    } else {
      ++it;
    }
  }

  std::unordered_set<int> activeTruthIds;
  for (const auto &target : truth) {
    auto &trail = truthTrails_[target.id];
    trail.emplace_back(target.state.head<2>());
    if (trail.size() > kTrailLength) {
      trail.pop_front();
    }
    activeTruthIds.insert(target.id);
  }
  for (auto it = truthTrails_.begin(); it != truthTrails_.end();) {
    if (activeTruthIds.find(it->first) == activeTruthIds.end()) {
      it = truthTrails_.erase(it);
    } else {
      ++it;
    }
  }
}

void ImGuiVisualizer::renderTrackTrails() const {
  for (const auto &entry : trackTrails_) {
    if (entry.second.size() < 2) {
      continue;
    }
    const auto color = colorVectorForStatus(TrackStatus::Confirmed);
    std::vector<Eigen::Vector2d> points(entry.second.begin(), entry.second.end());
    renderTrajectory(points, color, 2.4f);
  }
}

void ImGuiVisualizer::renderTrajectories(const std::unordered_map<int, std::deque<Eigen::Vector2d>> &trails,
                                         const Eigen::Vector3d &color,
                                         float thickness) const {
  for (const auto &entry : trails) {
    if (entry.second.size() < 2) {
      continue;
    }
    std::vector<Eigen::Vector2d> points(entry.second.begin(), entry.second.end());
    renderTrajectory(points, color, thickness);
  }
}

void ImGuiVisualizer::shutdown() {
  if (gridVBO_) {
    glDeleteBuffers(1, &gridVBO_);
    gridVBO_ = 0;
  }
  if (gridVAO_) {
    glDeleteVertexArrays(1, &gridVAO_);
    gridVAO_ = 0;
  }
  if (gridProgram_) {
    glDeleteProgram(gridProgram_);
    gridProgram_ = 0;
  }
  if (pointVBO_) {
    glDeleteBuffers(1, &pointVBO_);
    pointVBO_ = 0;
  }
  if (pointVAO_) {
    glDeleteVertexArrays(1, &pointVAO_);
    pointVAO_ = 0;
  }

  if (ImGui::GetCurrentContext()) {
    const ImGuiIO &io = ImGui::GetIO();
    if (io.IniFilename && io.IniFilename[0] != '\0') {
      ImGui::SaveIniSettingsToDisk(io.IniFilename);
    }
  }

  ImGui_ImplOpenGL3_Shutdown();
  ImGui_ImplGlfw_Shutdown();
  ImGui::DestroyContext();

  if (window_) {
    glfwDestroyWindow(window_);
    window_ = nullptr;
  }
  glfwTerminate();
}

const VisualizerOptions &ImGuiVisualizer::options() const {
  return options_;
}

void ImGuiVisualizer::setOptions(const VisualizerOptions &options) {
  options_ = options;
}

bool ImGuiVisualizer::shouldClose() const {
  return shouldClose_;
}

} // namespace rfs

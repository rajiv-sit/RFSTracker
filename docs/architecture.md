# RFSTracker Architecture

## 1. Scenario, Goals, and User Controls

- **Surveillance envelope:** the default configuration monitors a 5 km × 5 km rectangle (`TrackerConfig::areaWidth/areaHeight`) with a single radar located at the origin and assumed floor height (z = 0 for this 2D prototype). All measurements, clutter, and tracks are generated inside that area, and targets that leave are automatically dropped.
- **Targets & lifetimes:** the simulation can host up to `TrackerConfig::maxTargets` (default 50) with distinct `startTime`/`endTime` entries. Each `TargetDescriptor` contains an ID, initial kinematics, and a `maneuverInterval` that injects small heading changes to avoid trivial straight-line motion. Targets are reflected off the boundaries whenever they reach the edges.
- **Iteration control:** every run follows `TrackerConfig::samplingTime` and `maxSteps`. Both knobs are user-definable through JSON config files (e.g., `config/default_tracker_config.json`) or command-line overrides, giving precise control over scan duration and sampling density. `TrackerConfig::validate()` ensures the time step, spatial bounds, and `maxTargets` remain strictly positive, and that at least one sensor is configured.
- **Truth & measurement noise:** sensors define detection probability, false alarm rate, measurement range, azimuth/range standard deviations, and SNR. Real targets carry the higher configured SNR, while clutter is generated with a SNR that is typically 10 dB lower. All truth states (`TargetState_t`) and measurements (`MeasurementSet_t`) are stored in memory per scan; nothing is serialized to disk unless the user adds logging or exports the vectors manually.

## 2. Directory Layout

```
RFSTracker/
├── build_debug/                    # Debug build tree (build_debug.bat also launches rfs_app.exe here)
├── build_release/                  # Release build tree (build_release.bat behaves similarly)
├── cmake/                          # CMake tooling/presets
├── config/                         # JSON config files (default_tracker_config.json, user overrides)
├── docs/
│   └── architecture.md             # this document, kept aligned with the code
├── include/                        # public headers for config, simulation, filters, tracking, visualization
├── src/                            # implementation mirroring include/
├── shaders/                        # runtime shader files (`grid.vs`, `grid.fs`, others as needed)
├── bindings/                       # local ImGui backend sources (capture the same version as the imgui docking package)
├── vendor/
│   └── splinter/                   # Splinter B-spline toolkit (order-3 continuity, builder/generator code)
├── tests/
│   └── unit/                       # GoogleTest suites
├── build_debug.bat                 # runs Conan, CMake, builds, and launches rfs_app.exe for Debug
├── build_release.bat               # same for Release
├── CMakeLists.txt
└── conanfile.py
```

## 3. Configuration & System Models

- `TrackerConfig` centralizes every runtime parameter: sampling time, number of scans, spatial area, clutter/SNR budgets, visualization enable flag, the sensor list, the target descriptors, the selected `FilterFamily`, and the representation backend.
- Sensors are defined via `SensorConfig`. The radar sensor carries detection probability, false alarm rate, range, azimuth/range standard deviations, and signal-to-noise ratio. Adjusting these values simulates different sensor fidelity or clutter environments.
- Targets appear only while the current time is within `[startTime, endTime]`. The user can configure start/end times independently per target, so the scene can gracefully add or remove tracks at any point in the run. `maxTargets` caps simultaneous identities to avoid overload.
- When `representation` is `Particles`, set `core_particle_type` to `SIS`, `SIR`, `APF`, or `RPF` to select the bootstrap, auxiliary, or regularized weighting/resampling strategy; omitting the field defaults to SIR.

## 4. Truth Generation & Measurements

- `GroundTruthGenerator` keeps each `TargetDescriptor` and determines whether it is active at the current time. If so, it instantiates or updates a `TargetState_t`, propagates it with constant velocity, turns periodically according to `maneuverInterval`, and reflects velocities at the surveillance boundaries.
- `SimulationEnvironment` owns the truth generator and all configured `SensorInterface`s. Each `step(currentTime, dt, measurementSet)` advances truths, clears the previous `MeasurementSet_t`, and asks every sensor to `sense` the scene. `RadarSensor` draws detections via `std::bernoulli_distribution`, injects Gaussian noise based on the configured standard deviations, tags each `Measurement_t` with sensor metadata (`sensorId`, `truthId`, `snrDb`, `isClutter`), and fills the rest of the scene with Poisson-distributed clutter (positions drawn uniformly across the 5 km × 5 km window).
- Truth states and measurements are stored in memory and passed directly to the filter/track manager/visualizer every iteration. There is no default persistence—any file writes (e.g., logs) must be added explicitly.

## 5. Representations & Filtering

- `RepresentationFactory::createRepresentation` returns one of three backends:
  * `GaussianRepresentation`: maintains Gaussian components, inflates covariances during predict, prunes components below a weight threshold, and merges spatially close components after updates.
  * `ParticleRepresentation`: mutates particles’ velocities with Gaussian noise, applies measurement likelihoods to compute weights, normalizes them, and performs resampling when necessary.
  * `SplineRepresentation`: accumulates timestamped measurements and delegates to the Splinter toolkit (`vendor/splinter/`) to build an order-3 B-spline with continuous derivatives. The spline is evaluated to produce smoothed position/velocity estimates.
- Each filter family (`PhdFilter`, `CphdFilter`, `MbFilter`, `GlmbFilter`) wraps a `Representation`. `TrackingPipeline::instantiateFilter()` selects the desired family so switching between PHD, CPHD, MB, or GLMB is a single `TrackerConfig` change.

## 6. Association & Tracks

- `TrackManager` stores `TrackState` entries (ID, position, velocity, hit count, miss count, status, and last update time). Any unassociated measurement spawns a newborn track; consecutive hits raise the status to tentative/confirmed, while multiple misses result in pruning.
- `HungarianSolver` provides optimal assignment when matching tracks to measurements. The solver pads the cost matrix for unmatched rows/columns and returns assignments plus the minimal cost.
- Track metadata is available for display every scan. The GUI shows IDs, statuses, positions, velocities, hits, misses, and connection to measurements.

## 7. Tracking Pipeline Flow

The `TrackingPipeline` loops through these steps:
1. `SimulationEnvironment::step` generates the truth tuples plus the new `MeasurementSet_t`.
2. The selected filter executes `predict(dt)` and then `update(measurementSet_)`.
3. `TrackManager::update` handles associations, creates new tracks, and culls stale ones.
4. `PerformanceEvaluator` recomputes NEES, RMSE, and OSPA by comparing the current estimates to the truth vector.
5. The `Visualizer` (via runtime polymorphism) receives measurements, tracks, truth, metrics, scan ID, and elapsed time to paint the next frame.
6. The loop increments `currentTime_` and `scanId_`. If the GUI window closes, `visualizer_->shouldClose()` returns `true` and the loop exits gracefully.

## 8. Visualization & GUI

- `Visualizer` defines the runtime polymorphic contract (`initialize`, `renderFrame`, `shutdown`, `options`, `setOptions`, `shouldClose`). This opens the door for future heads-up-display visualizers without modifying the pipeline.
- `ImGuiVisualizer` is the concrete GUI implementation:
  * Initializes GLFW (OpenGL 3.3 core profile), GLAD, and ImGui using the local `bindings/imgui_impl_glfw.*` and `bindings/imgui_impl_opengl3.*` sources to keep the backend code synchronized with the configured `imgui/cci.20230105+1.89.2.docking` package.
  * Compiles vertex/fragment shaders from the `shaders/` directory (`grid.vs`, `grid.fs`) and prepares grid vertex buffers plus point buffers for measurements, truths, and tracks.
  * Maintains historical traces of RMSE, NEES, and OSPA (capped at ~256 samples) and renders them with ImGui’s `PlotLines`.
  * Displays a control panel that shows scan ID, elapsed time, truth/measurement/track counts, the `filterFamily`/`representation`, and — when `representation` is `Particles` — the configured `core_particle_type` (SIS/SIR/APF/RPF). Toggle controls allow the user to show/hide truth, measurements, and track overlays, and a dedicated button opens/closes the track-detail overlay.
  * The overlay lists every track’s ID, status, position, velocity, hits, and misses for the current scan.
  * Draws truth points (green), measurements (blue), newborn tracks (yellow), tentative tracks (pink), and confirmed tracks (red) with a shared shader-driven point renderer. The point size and colors are configurable via new Visualizer options if needed.
  * Exposes performance metrics (textual values plus sparklines) for NEES, RMSE, and OSPA on every frame.
  * Polls GLFW events, checks `glfwWindowShouldClose`, renders ImGui draw data, and swaps buffers. Closing the window sets `shouldClose_`, which stops the pipeline loop.
  * Cleans up OpenGL buffers, the shader program, ImGui, GLFW, and any allocated resources inside `shutdown()`.

## 9. Performance Metrics

- `PerformanceEvaluator` computes:
  * **NEES** – normalized squared error averaged over the first four components of the state.
  * **RMSE** – Euclidean root-mean-square of the 2D position estimate errors.
  * **OSPA** – OSPA distance with a 50 m cutoff, so unmatched tracks incur a fixed penalty.
- The computed metrics are fed to the visualizer for rendering and to future logging or evaluation hooks.

## 10. Builds & Tests

- `conanfile.py` (Conan 2.x) specifies the dependency stack: GLFW 3.4, Glew 2.2.0, GLU/system, GLM, the ImGui docking package (`imgui/cci.20230105+1.89.2.docking`), libcurl 8.9.1, libjpeg 9f, nanoflann 1.6.0, opengl/system, Eigen 3.4.0, fmt 10.2.1, spdlog 1.12.0, nlohmann_json 3.11.2, glad 0.1.36, and gtest 20210126. This combination provides the math, logging, visualization, and testing primitives.
- `CMakeLists.txt` consumes the generated `conan_toolchain.cmake`, globs `src/*.cpp`, `vendor/splinter/src/*.cpp`, and `bindings/*.cpp`, adds `include`, `vendor/splinter/include`, and `bindings` to the include path, and links `Eigen3`, `nlohmann_json`, `fmt`, `spdlog`, `imgui`, `glfw`, and `glad`. Windows builds get the `/FS` compile option to avoid file locking during parallel builds.
- `build_debug.bat` / `build_release.bat` run Conan, configure CMake, build the project, and finally drop into the appropriate build tree to launch `rfs_app.exe`. Running either script automatically starts the GUI, which must be closed manually to finish the script.
- `tests/unit/rfs_unit_tests.cpp` packs GoogleTest cases around config parsing, numerics, representations, association, track management, and performance metrics. They help keep the core logic covered, and more tests can be added progressively.

## 11. Extensibility Notes

- **Noise tuning:** `SensorConfig` encapsulates detection probability, clutter rate, range, noise standard deviations, and SNR, so users can dial in sensor fidelity at runtime.
- **Representation flexibility:** JSON can select `GaussianMixture`, `Particles`, or `Spline`. The spline backend relies on the `vendor/splinter/` builder (order-3 continuity) so Splinter stays under version control as part of the repo.
- **Filter families:** Supports PHD, CPHD, MB, and GLMB; new families can plug into `TrackingPipeline::instantiateFilter()` by inheriting `RepresentationFilter`.
- **Visualization modularity:** The pipeline treats the visualizer as a runtime interface, so future renderers (console, remote, headless) can replace `ImGuiVisualizer` without touching the simulation or filter code.

## 12. Validation & Logs

- Running `build_debug.bat` replays the Debug workflow (Conan install, CMake configure/build, and `rfs_app.exe` launch). The script drops into `build_debug\Debug`, so closing the GUI window is required to let the batch file finish; this also proves the ImGui + GLSL bindings continue to work with the local shader sources.
- With `TrackerConfig::logger_verbose` enabled, the app generates per-component log files next to the executable (`filter_debug.log`, `association_debug.log`, `track_manager_debug.log`, `representation_debug.log`, `rfs_app_debug.log`, etc.). These logs record scan IDs, prediction/update/association actions, and track lifecycle decisions and are already ignored via `.gitignore`, so they can be re-created freely when you need to trace a regression.
- `build_debug\Debug\rfs_unit_tests.exe` runs the GoogleTest suites for config parsing, numerics, representations, Hungarian assignment, track management, and performance metrics — expect `[  PASSED  ] 10 tests.` before pushing changes.
- Whenever you tweak clutter, detection, or SNR parameters (or any state-dependent sensor settings), rerun both `build_debug.bat` (to regen the logs and GUI visuals) and the unit tests so the logs keep showing one confirmed track per moving target and no phantom IDs linger, and so the test suite still passes in the new noise regime.

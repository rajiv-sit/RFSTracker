# RFSTracker

RFSTracker is a Windows-based multi-target tracking prototype that simulates a 5 km × 5 km surveillance area, fuses radar detections, runs configurable filters (PHD/CPHD/MB/GLMB) with Gaussian, particle, or B-spline representations, and visualizes everything through an ImGui-powered GUI (truth, measurements, color-coded tracks, and performance metrics).

## Key Features

- **Simulation control** – user-defined sampling time, number of scans, sensor & target descriptors, clutter/SNR budgets, and maximum simultaneous targets.
- **Filter families** – switch between PHD, CPHD, MB, or GLMB via `TrackerConfig` without touching the core pipeline; each family instantiates a representation backend (Gaussian mixture, particles, or Splinter B-spline order-3).
- **Tracks & metrics** – Hungarian assignment manages track lifecycles; PerformanceEvaluator reports NEES, RMSE, and OSPA each iteration for downstream evaluation or GUI sparkline panels.
- **Visualizer** – GLFW + ImGui + custom shaders render the grid, truths (green), measurements (blue), and tracks (yellow newborn, pink tentative, red confirmed). Toggle truth/measurement/track overlays, open the track detail table (ID, status, position, velocity, hits, misses), and watch the metric plots update in real time.
- **Vendor tooling** – Splinter B-spline sources live in `vendor/splinter/`, and ImGui GLFW/OpenGL bindings are compiled from `bindings/` so the GUI runs without conflicting OpenGL loaders.

## Requirements

- Windows 10/11 with Visual Studio 2022 (MSVC v143 toolset) or equivalent with C++20 support.
- Conan 2.x and CMake 3.23+ available on `PATH` (or bundle them via the launcher).
- `build_debug.bat` / `build_release.bat` expect `conan`, `cmake`, and `msbuild` available; the scripts install dependencies, configure CMake, build the solutions, and launch `rfs_app.exe`.
- OpenGL 3.3 capable GPU (needed for the ImGui visualizer + custom shaders).

## Quick Start (Debug)

1. Open a PowerShell prompt in the repository root.
2. Run `build_debug.bat`. It will:
   * Run `conan install` (Debug).
   * Configure & build the Debug CMake configuration.
   * Launch `build_debug\Debug\rfs_app.exe` automatically. Close the GUI window once you’ve reviewed the visuals to return to the shell.
3. `rfs_app.exe` uses `config/default_tracker_config.json` by default; if the file is missing, defaults are logged (you can pass a custom path on the command line).
   * CMake now copies the `config/` folder into each build output directory, so the executable always finds the JSONs (no manual copying required).

## Running the App

- The GUI shows the scan ID, elapsed time, truth/measurement/track counts, and performance metrics for every scan.
- Use the toggles to show/hide truth, measurement, and track overlays, and click “Show Track Details” to open the per-track table. The Tracker Control panel also lists the active `filterFamily`/`representation`, and when `representation` is `Particles` it surfaces the configured `core_particle_type` (`SIS`, `SIR`, `APF`, `RPF`) so you can confirm which particle strategy the GUI is driving.
- The control panel exposes NEES, RMSE, and OSPA alongside sparkline plots (256-sample history).
- Visualizer colors:
  * Truth: green
  * Measurements: red
  * Track statuses - confirmed (blue)

<figure>
  <img width="1917" height="1025" alt="image" src="https://github.com/user-attachments/assets/e374debb-c11e-46c8-9546-4bec98f85b4e" />
  <figcaption><strong>Figure 1.</strong> ImGui visualizer showing measurement dots, truth targets, confirmed-track labels, and the control/metrics panels.</figcaption>
</figure>

## Configuration

- Modify `config/default_tracker_config.json` (copy it from the template or create a new JSON) to:
  * define `samplingTime`, `maxSteps`, `maxTargets`, `areaWidth`, `areaHeight`
  * list `SensorConfig` entries (detection probability, false alarm rate, range, noise, SNR)
  * declare `TargetDescriptor`s with `id`, `startTime`, `endTime`, `initialState`, `maneuverInterval`
  * set `filterFamily` to `PHD`, `CPHD`, `MB`, or `GLMB` and `representation` to `GaussianMixture`, `Particles`, or `Spline`
    * when `representation` is `Particles`, add `core_particle_type` (`SIS`, `SIR`, `APF`, `RPF`) to pick the underlying particle filter strategy; omitting it defaults to `SIR`
    * optional `truthMatchThreshold` (meters) caps how far an estimate may drift from a truth state before the pipeline stops pairing them for logging/metrics; defaults to `80.0`

- Run `rfs_app.exe custom_config.json` from `build_debug\Debug` / `build_release\Release` to override the default file.

## Tests & Validation

- `./build_debug.bat` replays the full Debug workflow (Conan install, CMake configure/build, and it launches `rfs_app.exe`; close the GUI window to finish). It also creates `build_debug/Debug/filter_debug.log`, `association_debug.log`, `track_manager_debug.log`, and `rfs_app_debug.log` locally (all ignored via `.gitignore`), which are handy for tracing per-scan predictions/assignments when `logger_verbose` is `true`.
- Run `build_debug\Debug\rfs_unit_tests.exe` to validate config parsing, numerics, representations, Hungarian assignment, track management, and performance metrics; expect `[  PASSED  ] 10 tests.` in the output before pushing.
- Whenever you change clutter, detection, or SNR parameters, rerun both the GUI (`build_debug.bat`) and the unit tests so the logs keep showing one confirmed track per moving target and no ghost IDs linger.

## Architecture & Documentation

- `docs/architecture.md` walks through the scenario model, directories, configurations, simulations, representations, tracking pipeline, visualization, metrics, builds/tests, extensibility points, and validation steps. Keep it updated whenever you add sensors, filters, or UI features.

## Additional Notes

- The visualizer uses the local `bindings/` folder for `imgui_impl_glfw.*` and `imgui_impl_opengl3.*` so we control the GL loader. `IMGUI_IMPL_OPENGL_LOADER_CUSTOM` points to GLAD to avoid duplicate OpenGL headers.
- Splinter (order-3 B-spline builder) lives under `vendor/splinter/`. Its sources are compiled directly into `rfs_core`.
- `build_release.bat` behaves the same as `build_debug.bat` but targets Release; both scripts launch `rfs_app.exe`, so you can run the GUI right after building.

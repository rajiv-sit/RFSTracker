@echo off
REM Build RFSTracker in Debug configuration.
if not exist build_debug (
  mkdir build_debug
)
conan install . --output-folder=build_debug --build=missing -s build_type=Debug
cmake -S . -B build_debug -DCMAKE_BUILD_TYPE=Debug
cmake --build build_debug --config Debug
pushd build_debug\Debug
.\\rfs_app.exe
popd

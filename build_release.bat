@echo off
REM Build RFSTracker in Release configuration.
if not exist build_release (
  mkdir build_release
)
conan install . --output-folder=build_release --build=missing -s build_type=Release
cmake -S . -B build_release -DCMAKE_BUILD_TYPE=Release
cmake --build build_release --config Release
pushd build_release\Release
.\\rfs_app.exe
popd

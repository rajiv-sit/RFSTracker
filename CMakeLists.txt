cmake_minimum_required(VERSION 3.20)
project(RFSTracker LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

include(${CMAKE_BINARY_DIR}/conan_toolchain.cmake)

find_package(Eigen3 CONFIG REQUIRED)
find_package(spdlog CONFIG REQUIRED)
find_package(fmt CONFIG REQUIRED)
find_package(nlohmann_json CONFIG REQUIRED)
find_package(imgui CONFIG REQUIRED)
find_package(glfw3 CONFIG REQUIRED)
find_package(glad CONFIG REQUIRED)
find_package(GTest CONFIG REQUIRED)

file(GLOB_RECURSE RFSTRACKER_SOURCES CONFIGURE_DEPENDS src/*.cpp)
file(GLOB SPLINTER_SOURCES vendor/splinter/src/*.cpp)
file(GLOB BINDING_SOURCES bindings/*.cpp)
list(APPEND RFSTRACKER_SOURCES ${SPLINTER_SOURCES} ${BINDING_SOURCES})

add_library(rfs_core ${RFSTRACKER_SOURCES})
target_include_directories(rfs_core PUBLIC
    include
    vendor/splinter/include
    bindings
    ${Eigen3_INCLUDE_DIRS}
    ${nlohmann_json_INCLUDE_DIRS}
)
target_link_libraries(rfs_core PUBLIC
    Eigen3::Eigen
    nlohmann_json::nlohmann_json
    fmt::fmt
    spdlog::spdlog
    imgui::imgui
    glfw
    glad::glad
)
target_compile_definitions(rfs_core PRIVATE RFSTRACKER_SHADER_DIR="${CMAKE_SOURCE_DIR}/shaders")
if (MSVC)
    target_compile_options(rfs_core PRIVATE /FS)
endif()

add_executable(rfs_app src/main.cpp)
target_link_libraries(rfs_app PRIVATE rfs_core
    imgui::imgui
    glfw
)

add_executable(rfs_config_test tests/unit/test_tracker_config.cpp)
target_link_libraries(rfs_config_test PRIVATE rfs_core)

add_executable(rfs_unit_tests tests/unit/rfs_unit_tests.cpp)
target_link_libraries(rfs_unit_tests PRIVATE rfs_core gtest::gtest)

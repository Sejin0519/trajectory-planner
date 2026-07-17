cmake_minimum_required(VERSION 3.16)
project(trajectory_planner CXX)
project(trajectory_planner C CXX)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(raylib REQUIRED)

file(GLOB IMGUI_SOURCES external/imgui/*.cpp)
file(GLOB IMPLOT_SOURCES external/implot/*.cpp)

add_executable(planner
    src/main.cpp
    src/trajectory.cpp
    src/cubicBezier.cpp
    src/pathManager.cpp
    src/util.hpp
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
    external/rlimgui/rlImGui.cpp
    external/tinyfd/tinyfiledialogs.c
)

target_include_directories(planner PRIVATE external/imgui external/rlimgui external/implot external/tinyfd)
target_compile_options(planner PRIVATE -Wall -Wextra)
target_link_libraries(planner raylib)
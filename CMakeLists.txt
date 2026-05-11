cmake_minimum_required(VERSION 3.16)
project(RadarSim CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Warnings ──────────────────────────────────────────────────────────────────
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Sources ───────────────────────────────────────────────────────────────────
set(CORE_SOURCES
    core/Entity.cpp
    core/Radar.cpp
    core/KalmanFilter1D.cpp
    core/Tracker.cpp
)

add_executable(radar_sim main.cpp ${CORE_SOURCES})
target_include_directories(radar_sim PRIVATE ${CMAKE_SOURCE_DIR})

# ── Build instructions ────────────────────────────────────────────────────────
# mkdir build && cd build
# cmake ..
# cmake --build .
# ./radar_sim
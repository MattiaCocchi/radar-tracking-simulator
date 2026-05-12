cmake_minimum_required(VERSION 3.16)
project(FrictionEstimator CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ── Warnings per un codice professionale ──────────────────────────────────────
if(MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ── Nuovi Sorgenti Core ───────────────────────────────────────────────────────
# Nota: Abbiamo sostituito i vecchi file del radar con il nuovo motore fisico
set(CORE_SOURCES
    core/ExtendedKalmanFilter.cpp
    core/FrictionSimulator.cpp
    # WheelModel.h e TelemetryLogger.h sono header-only, 
    # quindi non serve elencarli qui tra i .cpp
)

# ── Eseguibile ────────────────────────────────────────────────────────────────
add_executable(friction_estimator main.cpp ${CORE_SOURCES})

# Questo permette al compilatore di trovare gli header dentro la cartella /core
target_include_directories(friction_estimator PRIVATE ${CMAKE_SOURCE_DIR})

# ── Istruzioni per la Build ───────────────────────────────────────────────────
# mkdir build && cd build
# cmake ..
# cmake --build .
# ./friction_estimator
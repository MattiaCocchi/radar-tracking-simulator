cmake_minimum_required(VERSION 3.10)
project(RadarSimulation)

# Impostiamo lo standard C++ (il 17 è perfetto per quello che facciamo)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Diciamo a CMake dove trovare i file .h (la cartella core)
include_directories(core)

# Elenchiamo tutti i file .cpp
set(SOURCES 
    main.cpp 
    core/Entity.cpp 
    core/Radar.cpp 
    core/Tracker.cpp
)

# Creiamo l'eseguibile
add_executable(radar_sim ${SOURCES})
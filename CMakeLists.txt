cmake_minimum_required(VERSION 3.16)
project(swarm_sim LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Help CMake find MinGW if installed in common locations
if(WIN32 AND NOT MSVC)
    set(CMAKE_C_COMPILER "gcc")
    set(CMAKE_CXX_COMPILER "g++")
endif()

add_executable(swarm_sim
    src/main.cpp
    src/formations.cpp
)

target_include_directories(swarm_sim PRIVATE src)

if (MSVC)
    target_compile_options(swarm_sim PRIVATE /W4)
else()
    target_compile_options(swarm_sim PRIVATE -Wall -Wextra -Wpedantic)
endif()

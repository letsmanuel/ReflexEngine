cmake_minimum_required(VERSION 3.20)

project(ReflexEngine
    VERSION 0.1.0
    DESCRIPTION "Procedural, physics-driven character reaction engine"
    HOMEPAGE_URL "https://github.com/letsmanuel/ReflexEngine"
    LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(PROJECT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    set(REFLEX_IS_TOP_LEVEL ON)
else()
    set(REFLEX_IS_TOP_LEVEL OFF)
endif()

option(REFLEX_BUILD_SHARED "Build the engine as a shared library (DLL / .so / .dylib)" OFF)
option(REFLEX_BUILD_TESTS  "Build unit and scenario tests" ${REFLEX_IS_TOP_LEVEL})
option(REFLEX_BUILD_DEMO   "Build the raylib demo application" ${REFLEX_IS_TOP_LEVEL})

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "" FORCE)
endif()

# Use the dynamic MSVC runtime everywhere so Jolt, raylib and tests agree.
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>DLL")

include(cmake/Dependencies.cmake)
add_subdirectory(src)

if(REFLEX_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
    add_subdirectory(tools)
endif()

if(REFLEX_BUILD_DEMO)
    add_subdirectory(demo)
endif()

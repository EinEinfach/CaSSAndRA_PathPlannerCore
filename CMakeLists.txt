cmake_minimum_required(VERSION 3.15)
project(CoveragePlanner LANGUAGES CXX)

# Unterdrückt die CMP0148 Warnung (nutze das neue Python-Modul)
set(PYBIND11_FINDPYTHON ON)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 1. Header-Pfade
include_directories(include)

# 2. Alle Quellen in src inkl. Unterordner (Clipper2) finden
file(GLOB_RECURSE ALL_SOURCES "src/*.cpp")

# Trenne Core-Sourcen vom Python-Wrapper
set(CORE_SOURCES ${ALL_SOURCES})
list(FILTER CORE_SOURCES EXCLUDE REGEX "python_wrapper.cpp")

# 3. Target: Standalone App
add_executable(CoveragePlanner main.cpp ${CORE_SOURCES})

# 4. Target: Python Modul
find_package(pybind11 REQUIRED)
pybind11_add_module(planner_module src/python_wrapper.cpp ${CORE_SOURCES})

# 5. Plattform-Spezifische Einstellungen
if(APPLE)
    # Falls du Homebrew nutzt, hilft das CMake die Pfade zu finden
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
    include_directories(/opt/homebrew/include)
else()
    # Für Linux / Raspberry Pi
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# 6. Google Tests
enable_testing()
find_package(GTest)
if(GTest_FOUND)
    # Sucht alle .cpp Dateien im tests/ Ordner automatisch
    file(GLOB TEST_SOURCES "tests/*.cpp")
    
    if(TEST_SOURCES)
        add_executable(run_tests ${TEST_SOURCES} ${CORE_SOURCES})
        target_link_libraries(run_tests GTest::gtest_main GTest::gtest)
    endif()
endif()
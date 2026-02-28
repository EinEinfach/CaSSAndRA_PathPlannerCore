cmake_minimum_required(VERSION 3.15)
project(CoveragePlanner LANGUAGES CXX)

# Unterdrückt die CMP0148 Warnung & nutzt das moderne FindPython3
set(PYBIND11_FINDPYTHON ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 1. Python & pybind11 Suche (Venv-aware)
# -----------------------------------------------------------------------------

# Falls -DPython_EXECUTABLE beim Aufruf nicht mitgegeben wurde, 
# versuchen wir die .venv im Projektverzeichnis zu nutzen
if(NOT Python_EXECUTABLE AND EXISTS "${CMAKE_SOURCE_DIR}/.venv/bin/python3")
    set(Python_EXECUTABLE "${CMAKE_SOURCE_DIR}/.venv/bin/python3")
endif()

find_package(Python3 REQUIRED COMPONENTS Interpreter Development)

# Trick: pybind11 fragen, wo seine CMake-Dateien liegen
execute_process(
    COMMAND "${Python3_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
    OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

if(PYBIND11_CMAKE_DIR)
    list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
endif()

find_package(pybind11 REQUIRED)

# -----------------------------------------------------------------------------
# 2. Quellen & Header
# -----------------------------------------------------------------------------
include_directories(include)

# Alle Quellen finden
file(GLOB_RECURSE ALL_SOURCES "src/*.cpp")

# Trenne Core-Sourcen vom Python-Wrapper
set(CORE_SOURCES ${ALL_SOURCES})
list(FILTER CORE_SOURCES EXCLUDE REGEX "python_wrapper.cpp")

# -----------------------------------------------------------------------------
# 3. Targets
# -----------------------------------------------------------------------------

# Standalone App
add_executable(CoveragePlanner main.cpp ${CORE_SOURCES})

# Python Modul
pybind11_add_module(planner_module src/python_wrapper.cpp ${CORE_SOURCES})

install(TARGETS planner_module LIBRARY DESTINATION .)

# -----------------------------------------------------------------------------
# 4. Plattform-Spezifische Einstellungen
# -----------------------------------------------------------------------------
if(APPLE)
    list(APPEND CMAKE_PREFIX_PATH "/opt/homebrew")
    include_directories(/opt/homebrew/include)
else()
    # Für Linux / Raspberry Pi
    set(CMAKE_POSITION_INDEPENDENT_CODE ON)
endif()

# -----------------------------------------------------------------------------
# 5. Google Tests
# -----------------------------------------------------------------------------
enable_testing()
find_package(GTest)
if(GTest_FOUND)
    file(GLOB TEST_SOURCES "tests/*.cpp")
    if(TEST_SOURCES)
        add_executable(run_tests ${TEST_SOURCES} ${CORE_SOURCES})
        target_link_libraries(run_tests GTest::gtest_main GTest::gtest)
    endif()
endif()
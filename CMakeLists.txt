cmake_minimum_required(VERSION 3.15)
project(CoveragePlanner LANGUAGES CXX)

# Unterdrückt die CMP0148 Warnung & nutzt das moderne FindPython3
set(PYBIND11_FINDPYTHON ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# -----------------------------------------------------------------------------
# 1. Python & pybind11 Suche (Venv-aware)
# -----------------------------------------------------------------------------

# -----------------------------------------------------------------------------
# 1. Python & pybind11 Suche
# -----------------------------------------------------------------------------

if(NOT SKBUILD)
    # Lokaler Build: Suche Python wie bisher (für venv)
    if(NOT Python_EXECUTABLE AND EXISTS "${CMAKE_SOURCE_DIR}/.venv/bin/python3")
        set(Python_EXECUTABLE "${CMAKE_SOURCE_DIR}/.venv/bin/python3")
    endif()
    find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
    
    # pybind11 Pfad lokal ermitteln
    execute_process(
        COMMAND "${Python3_EXECUTABLE}" -c "import pybind11; print(pybind11.get_cmake_dir())"
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_QUIET
    )
    if(PYBIND11_CMAKE_DIR)
        list(APPEND CMAKE_PREFIX_PATH "${PYBIND11_CMAKE_DIR}")
    endif()
else()
    # Wheel-Build via scikit-build-core: 
    # Wir brauchen find_package(Python3) hier GAR NICHT, 
    # da scikit-build-core die Pfade bereits gesetzt hat.
    message(STATUS "SKBUILD erkannt: Überspringe manuelle Python-Suche")
endif()

# pybind11 wird immer benötigt
find_package(pybind11 REQUIRED)

# -----------------------------------------------------------------------------
# 2. Quellen & Header
# -----------------------------------------------------------------------------
include_directories(include)

# Wir suchen alle .cpp Dateien im src Ordner
file(GLOB_RECURSE ALL_SOURCES "src/*.cpp")

# Wir filtern den Wrapper und den Reader EXAKT aus
set(CORE_SOURCES "")
foreach(_source ${ALL_SOURCES})
    # Schließe den Python Wrapper und den GeoJSONReader aus den Core-Sourcen aus
    if(NOT _source MATCHES "python_wrapper.cpp" AND NOT _source MATCHES "GeoJSONReader.cpp")
        list(APPEND CORE_SOURCES ${_source})
    endif()
endforeach()

# -----------------------------------------------------------------------------
# 3. Targets
# -----------------------------------------------------------------------------

# Standalone App: Nur bauen, wenn wir NICHT für ein Python-Wheel (SKBUILD) kompilieren
if(NOT SKBUILD)
   message(STATUS "Standalone-Modus: Baue App und Tests...")
    # A. Standalone App (benötigt nlohmann/json und GeoJSONReader.cpp)
    set(APP_SOURCES ${CORE_SOURCES} "src/GeoJSONReader.cpp")
    # Hier nur add_executable, wenn die Header wirklich da sind
    add_executable(CoveragePlanner main.cpp ${APP_SOURCES})

    # B. Google Tests
    enable_testing()
    find_package(GTest)
    if(GTest_FOUND)
        file(GLOB TEST_SOURCES "tests/*.cpp")
        if(TEST_SOURCES)
            add_executable(run_tests ${TEST_SOURCES} ${CORE_SOURCES})
            target_link_libraries(run_tests GTest::gtest_main GTest::gtest)
        endif()
    endif()
endif()

# Python Modul: Wird immer gebaut (auch im Wheel)
pybind11_add_module(planner_module src/python_wrapper.cpp ${CORE_SOURCES})

# Wichtig für scikit-build-core
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
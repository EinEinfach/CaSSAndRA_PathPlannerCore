# Compiler Einstellungen
CXX = clang++
CXXFLAGS = -std=c++20 -g3 -O0 -Wall -Wextra -Iinclude -I/opt/homebrew/include -MMD -MP

# Pfade
SRC_DIR = src
CLIPPER_DIR = src/clipper2
BUILD_DIR = build
TARGET = $(BUILD_DIR)/CoveragePlanner

# 1. Liste alle Quellen mit ihren tatsächlichen Pfaden auf
CORE_SRCS = $(wildcard $(SRC_DIR)/*.cpp)
CLIPPER_SRCS = $(wildcard $(CLIPPER_DIR)/*.cpp)
ALL_SRCS = main.cpp $(CORE_SRCS) $(CLIPPER_SRCS)

# 2. Erzeuge Objekt-Pfade, die die Unterordner-Struktur im Build-Ordner spiegeln
# Dies verhindert Namenskollisionen und "No rule to make target" Fehler
OBJS = $(ALL_SRCS:%.cpp=$(BUILD_DIR)/%.o)
DEPS = $(OBJS:.o=.d)

# 3. Filter für die verschiedenen Targets
# Standalone braucht kein python_wrapper
STANDALONE_OBJS = $(filter-out $(BUILD_DIR)/$(SRC_DIR)/python_wrapper.o, $(OBJS))
# Python braucht kein main.o
PYTHON_OBJS = $(filter-out $(BUILD_DIR)/main.o $(BUILD_DIR)/$(SRC_DIR)/python_wrapper.o, $(OBJS))

# Standard-Target
all: $(TARGET)

# Linken der Standalone-App
$(TARGET): $(STANDALONE_OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(STANDALONE_OBJS)

# Regel für Dateien im Wurzelverzeichnis (main.cpp)
$(BUILD_DIR)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Regel für Dateien in src/ und src/clipper2/
$(BUILD_DIR)/$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(DEPS)

# Aufräumen
clean:
	rm -rf $(BUILD_DIR)
	rm -f planner_module*.so

.PHONY: all clean test python_module

# --- Test Konfiguration ---
TEST_DIR = tests
GTEST_FLAGS = -L/opt/homebrew/lib -lgtest -lgtest_main -lpthread
GTEST_INC = -I/opt/homebrew/include

test: $(PYTHON_OBJS)
	@mkdir -p $(BUILD_DIR)
	$(CXX) $(CXXFLAGS) $(GTEST_INC) $(wildcard $(TEST_DIR)/*.cpp) \
	$(PYTHON_OBJS) -o $(BUILD_DIR)/run_tests $(GTEST_FLAGS)
	./$(BUILD_DIR)/run_tests

######       PYTHON_MODULE      ######
VENV_BIN ?= .venv/bin
PY_INC = $(shell $(VENV_BIN)/python3 -m pybind11 --includes)
PY_SUFFIX = $(shell $(VENV_BIN)/python3-config --extension-suffix)

python_module: $(PYTHON_OBJS)
	$(CXX) $(CXXFLAGS) $(PY_INC) -shared -undefined dynamic_lookup \
	$(PYTHON_OBJS) src/python_wrapper.cpp \
	-o planner_module$(PY_SUFFIX)
# Compiler Einstellungen
CXX = clang++
# Wir bleiben bei C++20, da Apple Clang C++23 noch nicht voll unterstützt
# Füge -g unbedingt hinzu!
CXXFLAGS = -std=c++20 -g3 -O0 -Wall -Wextra -Iinclude -MMD -MP

# Pfade
SRC_DIR = src
BUILD_DIR = build
TARGET = $(BUILD_DIR)/CoveragePlanner

# Quellen finden
SRCS = main.cpp $(wildcard $(SRC_DIR)/*.cpp)

# Objektdateien (build/main.o, build/Geometry.o, etc.)
OBJS = $(addprefix $(BUILD_DIR)/, $(notdir $(SRCS:.cpp=.o)))

# Dateiabhängigkeiten (.d Dateien), damit Änderungen an .hpp erkannt werden
DEPS = $(OBJS:.o=.d)

# Standard-Target
all: $(BUILD_DIR) $(TARGET)

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

# Linken
$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

# Kompilieren von main.cpp (liegt im Wurzelverzeichnis)
$(BUILD_DIR)/main.o: main.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Kompilieren der Dateien aus src/
$(BUILD_DIR)/%.o: $(SRC_DIR)/%.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Binde die .d Dateien ein, falls sie existieren
-include $(DEPS)

# Aufräumen
clean:
	rm -rf $(BUILD_DIR)

.PHONY: all clean

# --- Test Konfiguration ---
TEST_DIR = tests
GTEST_FLAGS = -L/opt/homebrew/lib -lgtest -lgtest_main -lpthread
GTEST_INC = -I/opt/homebrew/include

# Alle Testdateien finden
TEST_SRCS = $(wildcard $(TEST_DIR)/*.cpp)

# Target für die Tests
test: $(BUILD_DIR) $(filter-out $(BUILD_DIR)/main.o, $(OBJS))
	$(CXX) $(CXXFLAGS) $(GTEST_INC) $(TEST_SRCS) $(filter-out $(BUILD_DIR)/main.o, $(OBJS)) -o $(BUILD_DIR)/run_tests $(GTEST_FLAGS)
	./$(BUILD_DIR)/run_tests

# # Pfade für Homebrew GoogleTest (auf Apple Silicon)
# GTEST_FLAGS = -L/opt/homebrew/lib -lgtest -lgtest_main -lpthread
# GTEST_INC = -I/opt/homebrew/include

# # Ein separates Target für die Tests
# test: $(BUILD_DIR) $(filter-out $(BUILD_DIR)/main.o, $(OBJS))
# 	$(CXX) $(CXXFLAGS) $(GTEST_INC) tests/GeometryTests.cpp $(filter-out $(BUILD_DIR)/main.o, $(OBJS)) -o $(BUILD_DIR)/run_tests $(GTEST_FLAGS)
# 	./$(BUILD_DIR)/run_tests
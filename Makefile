# Compiler
CXX = g++

# Compiler flags
CXXFLAGS = -std=c++11 -Wall -Wextra -Iinclude

# Linker flags
LDFLAGS = -lraylib

# Source directories
SRC_DIR = RAPID
TEST_DIR = tests

# Source files
SRC_FILES = $(wildcard $(SRC_DIR)/*.cpp)
# TEST_FILES = $(wildcard $(TEST_DIR)/*.cpp)

# Object files
OBJ_FILES = $(SRC_FILES:$(SRC_DIR)/%.cpp=$(SRC_DIR)/%.o)
# TEST_OBJ_FILES = $(TEST_FILES:$(TEST_DIR)/%.cpp=$(TEST_DIR)/%.o)

# Executable names
TARGET = builds/rapid_app
# TEST_TARGET = test_executable

# Default target
all: $(TARGET) #$(TEST_TARGET)

# Build the main application
$(TARGET): $(OBJ_FILES)
	$(CXX) $(OBJ_FILES) -o $@ $(LDFLAGS)

# Build the test executable
# $(TEST_TARGET): $(TEST_OBJ_FILES)
# 	$(CXX) $(TEST_OBJ_FILES) -o $(TEST_DIR)/$@ $(LDFLAGS)

# Compile source files into object files
$(SRC_DIR)/%.o: $(SRC_DIR)/%.cpp $(SRC_DIR)/utils.h
	$(CXX) $(CXXFLAGS) -c $< -o $@

# $(TEST_DIR)/%.o: $(TEST_DIR)/%.cpp $(SRC_DIR)/utils.h
# 	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean up generated files
clean:
# rm -f $(OBJ_FILES) $(TEST_OBJ_FILES) $(TARGET) $(TEST_DIR)/$(TEST_TARGET)
	rm -f $(OBJ_FILES) $(TEST_OBJ_FILES) $(TARGET)

.PHONY: all clean
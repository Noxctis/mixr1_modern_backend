CXX = g++
CXXFLAGS = -O3 -Wall -std=c++17 -Iinclude
LDFLAGS = -lpigpiod_if2 -lpthread

SRC_DIR = src
OBJ_DIR = obj
INC_DIR = include

# Find all .cpp files in src directory
SRCS = $(wildcard $(SRC_DIR)/*.cpp)
# Map .cpp files to .o files in obj directory
OBJS = $(patsubst $(SRC_DIR)/%.cpp,$(OBJ_DIR)/%.o,$(SRCS))
# Header-dependency files generated alongside each .o
DEPS = $(OBJS:.o=.d)
TARGET = mixr1_daemon

# Default target
all: $(TARGET)

# Link all object files into the final executable
$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Compile each .cpp file into a .o file.
# -MMD -MP records which headers each .cpp includes, so editing a header
# (e.g. include/config.hpp) automatically rebuilds everything that uses it.
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -MMD -MP -c $< -o $@

# Create obj directory if it doesn't exist
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Build, then run the daemon (sudo needed for SCHED_FIFO)
run: $(TARGET)
	sudo ./$(TARGET)

# Build, then run the PI tuning matrix (460 RPM, 20 s per run, 2 repeats)
matrix: $(TARGET)
	sudo tools/run_matrix.sh 460 20 2

# Clean up build artifacts
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

.PHONY: all clean run matrix

-include $(DEPS)
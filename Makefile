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
TARGET = mixr1_daemon

# Default target
all: $(TARGET)

# Link all object files into the final executable
$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

# Compile each .cpp file into a .o file
$(OBJ_DIR)/%.o: $(SRC_DIR)/%.cpp | $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Create obj directory if it doesn't exist
$(OBJ_DIR):
	mkdir -p $(OBJ_DIR)

# Clean up build artifacts
clean:
	rm -rf $(OBJ_DIR) $(TARGET)

.PHONY: all clean
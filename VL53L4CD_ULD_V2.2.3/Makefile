# Makefile - builds tof_cal (calibration tool) and mixr1 (fluid controller)
#
# Usage:
#   make          build both
#   make mixr1    build only the controller
#   make tof_cal  build only the calibration tool
#   make clean    remove build output

CC       := gcc
CXX      := g++
INCLUDES := -I. -IPlatform -IVL53L4CD_ULD_Driver
CFLAGS   := -O2 -Wall $(INCLUDES)
CXXFLAGS := -std=c++14 -O2 -Wall -Wextra $(INCLUDES) -MMD -MP

BUILD := build

# ST driver + platform layer are plain C
C_SRCS := VL53L4CD_ULD_Driver/VL53L4CD_api.c Platform/platform.c
C_OBJS := $(patsubst %.c,$(BUILD)/%.o,$(C_SRCS))

.PHONY: all clean
all: tof_cal mixr1

# --- executables ---------------------------------------------------------
tof_cal: $(BUILD)/tof_cal.o $(C_OBJS)
	$(CXX) $^ -o $@

mixr1: $(BUILD)/mixr1_fluid_controller.o $(C_OBJS)
	$(CXX) $^ -o $@ -lwiringPi

# --- compile rules -------------------------------------------------------
$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(BUILD) tof_cal mixr1

# automatic header dependencies (so editing a .h rebuilds what uses it)
-include $(wildcard $(BUILD)/*.d $(BUILD)/*/*.d)
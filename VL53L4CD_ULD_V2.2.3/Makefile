# Makefile for tof_cal (VL53L4CD calibration tool, Raspberry Pi 4B)
#
# Put this file next to tof_cal.cpp. It searches the folders below it for
# VL53L4CD_api.c, VL53L4CD_calibration.c and platform.c, so the exact
# sub-folder layout of the ST package does not matter.
#
# If it picks the wrong copy, override on the command line, e.g.:
#   make PLAT_C=Platform/platform.c

CC       := gcc
CXX      := g++

BUILD    := build
TARGET   := tof_cal

# Skips build/ and any *Example* folder (the STM32 CubeIDE example has its own platform.c)
find_one = $(firstword $(shell find . -name $(1) -not -path './$(BUILD)/*' -not -path '*Example*' 2>/dev/null))

API_C    := $(call find_one,VL53L4CD_api.c)
CAL_C    := $(call find_one,VL53L4CD_calibration.c)
PLAT_C   := $(call find_one,platform.c)

# Platform dir first so YOUR platform.h is the one found
INC_DIRS := $(sort $(dir $(PLAT_C))) $(sort $(dir $(API_C))) $(sort $(dir $(CAL_C)))
INCS     := $(addprefix -I,$(INC_DIRS)) -I.

C_SRCS   := $(API_C) $(CAL_C) $(PLAT_C)
CXX_SRCS := tof_cal.cpp

CFLAGS   := -O2 -Wall $(INCS)
CXXFLAGS := -O2 -Wall -std=c++17 $(INCS)
LDLIBS   := -lm

OBJS     := $(patsubst ./%,%,$(patsubst %.c,$(BUILD)/%.o,$(C_SRCS))) \
            $(patsubst %.cpp,$(BUILD)/%.o,$(CXX_SRCS))

.PHONY: all clean info run-stats

all: info $(TARGET)

# Shows what was found, and warns about duplicate platform.h copies
info:
	@echo "api.c         : $(API_C)"
	@echo "calibration.c : $(CAL_C)"
	@echo "platform.c    : $(PLAT_C)"
	@test -f "$(dir $(PLAT_C))platform.h" || (echo "ERROR: no platform.h next to $(PLAT_C). Use: make PLAT_C=<your platform.c>"; exit 1)
	@test -n "$(API_C)" -a -n "$(PLAT_C)" || (echo "ERROR: driver files not found under $$(pwd)"; exit 1)
	@n=$$(find . -name platform.h -not -path './$(BUILD)/*' -not -path '*Example*' | wc -l); \
	if [ $$n -gt 1 ]; then echo "WARNING: $$n copies of platform.h found:"; \
	find . -name platform.h -not -path './$(BUILD)/*' -not -path '*Example*'; \
	echo "The one next to VL53L4CD_api.h wins; make sure it is your modified one."; fi

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDLIBS)

$(BUILD)/%.o: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

run-stats: $(TARGET)
	sudo ./$(TARGET) stats

clean:
	rm -rf $(BUILD) $(TARGET)
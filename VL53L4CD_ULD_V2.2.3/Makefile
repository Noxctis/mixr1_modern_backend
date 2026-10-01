# Makefile for mixr1_fluid_controller
# Expected layout (same folder as this Makefile):
#   mixr1_fluid_controller.cpp
#   VL53L4CD_ULD_Driver/*.c, *.h
#   Platform/*.c (or .cpp), *.h

TARGET := mixr1_fluid_controller

CC  := gcc
CXX := g++

INCLUDES := -I. -IVL53L4CD_ULD_Driver -IPlatform
CFLAGS   := -O2 -Wall $(INCLUDES)
CXXFLAGS := -O2 -Wall -std=c++17 $(INCLUDES)
LDLIBS   := -lwiringPi -lpthread

C_SRCS   := $(wildcard VL53L4CD_ULD_Driver/*.c) $(wildcard Platform/*.c)
CXX_SRCS := mixr1_fluid_controller.cpp $(wildcard Platform/*.cpp)

OBJS := $(C_SRCS:.c=.o) $(CXX_SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(OBJS) -o $@ $(LDLIBS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)

.PHONY: all clean
# Makefile
CXX = g++
CXXFLAGS = -I. -Wall -Wextra -std=gnu++17 -O2 -Wno-register
LDFLAGS = -lwiringPi

TARGET1 = mixr1_water_suite
TARGET2 = mixr1_fluid_controller

SRCS1 = mixr1_water_suite.cpp VL53L0X.cpp I2Cdev.cpp
SRCS2 = mixr1_fluid_controller.cpp VL53L0X.cpp I2Cdev.cpp

OBJS1 = $(SRCS1:.cpp=.o)
OBJS2 = $(SRCS2:.cpp=.o)

all: \((TARGET1)\)(TARGET2)

\((TARGET1):\)(OBJS1)
	\((CXX) -o\)@ \(^\)(LDFLAGS)

\((TARGET2):\)(OBJS2)
	\((CXX) -o\)@ \(^\)(LDFLAGS)

%.o: %.cpp
	\((CXX)\)(CXXFLAGS) -c \(< -o\)@

clean:
	rm -f \((OBJS1)\)(OBJS2) \((TARGET1)\)(TARGET2)
# Makefile for MIXR-1 fluid controller with Pololu VL53L4CD
# Folder must contain: VL53L4CD.hpp, vl53l4cd_test.cpp, mixr1_fluid_controller_l4cd.cpp
#
#   make          build both programs
#   make test     sensor bring-up test only (no wiringPi needed)
#   make mixr1    the controller (needs wiringPi)
#   make clean

CXX      ?= g++
CXXFLAGS ?= -O2 -std=c++17 -Wall -Wextra
LDLIBS_MIXR1 = -lwiringPi -lpthread

.PHONY: all test mixr1 clean

all: vl53l4cd_test mixr1

test: vl53l4cd_test

mixr1: mixr1_fluid_controller_l4cd
	cp $< $@

vl53l4cd_test: vl53l4cd_test.cpp VL53L4CD.hpp
	$(CXX) $(CXXFLAGS) $< -o $@

mixr1_fluid_controller_l4cd: mixr1_fluid_controller_l4cd.cpp VL53L4CD.hpp
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDLIBS_MIXR1)

clean:
	rm -f vl53l4cd_test mixr1_fluid_controller_l4cd mixr1

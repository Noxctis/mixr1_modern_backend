CXX = g++
CXXFLAGS = -I. -Wall -Wextra -std=gnu++11 -O2 -Wno-register
LDFLAGS = -lwiringPi

TARGET = mixr1_water_suite
SRCS = mixr1_water_suite.cpp VL53L0X.cpp I2Cdev.cpp
OBJS = $(SRCS:.cpp=.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f $(OBJS) $(TARGET)
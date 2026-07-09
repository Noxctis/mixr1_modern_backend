#include "VL53L0X.hpp"

#include <chrono>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <unistd.h>

// SIGINT (CTRL-C) exit flag and signal handler
volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int) {
	exitFlag = 1;
}

int main() {
	// Configuration constants
	// Number of sensors. If changed, make sure to adjust pins and addresses accordingly (ie to match size).
	const int SENSOR_COUNT = 6;
	// GPIO pins to use for sensors' XSHUT. As exported by WiringPi.
	const uint8_t pins[SENSOR_COUNT] = { 0, 1, 2, 3, 4, 5 };
	// Sensors' addresses that will be set and used. These have to be unique.
	const uint8_t addresses[SENSOR_COUNT] = {
		VL53L0X_ADDRESS_DEFAULT + 2,
		VL53L0X_ADDRESS_DEFAULT + 4,
		VL53L0X_ADDRESS_DEFAULT + 6,
		VL53L0X_ADDRESS_DEFAULT + 10,
		VL53L0X_ADDRESS_DEFAULT + 12,
		VL53L0X_ADDRESS_DEFAULT + 14
	};

	// Register SIGINT handler
	signal(SIGINT, sigintHandler);

	// Create sensor objects' array
	VL53L0X* sensors[SENSOR_COUNT];

	// Create sensors (and ensure GPIO pin mode)
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		sensors[i] = new VL53L0X(pins[i]);
		sensors[i]->powerOff();
	}
	// Just a check for an early CTRL-C
	if (exitFlag) {
		return 0;
	}

	// For each sensor: create object, init the sensor (ensures power on), set timeout and address
	// Note: don't power off - it will reset the address to default!
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		try {
			// Initialize...
			sensors[i]->initialize();
			// ...set measurement timeout...
			sensors[i]->setTimeout(200);
			// ...set the lowest possible timing budget (high speed mode)...
			sensors[i]->setMeasurementTimingBudget(20000);
			// ...and set I2C address...
			sensors[i]->setAddress(addresses[i]);
			// ...also, notify user.
			std::cout << "Sensor " << i << " initialized, real time budget: " << sensors[i]->getMeasurementTimingBudget() << std::endl;
		} catch (const std::exception & error) {
			std::cerr << "Error initializing sensor " << i << " with reason:" << std::endl << error.what() << std::endl;
			return 1;
		}
	}

	// Start continuous back-to-back measurement
	for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
		try {
			sensors[i]->startContinuous();
		} catch (const std::exception & error) {
			std::cerr << "Error starting continuous read mode for sensor " << i << " with reason:" << std::endl << error.what() << std::endl;
			return 2;
		}
	}

	// Durations in nanoseconds
	uint64_t totalDuration = 0;
	uint64_t maxDuration = 0;
	uint64_t minDuration = 1000*1000*1000;
	// Initialize reference time measurement
	std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

	// We need that variable after the for loop
	int j = 0;
	// Also, set fill and width options for cout so that measurements are aligned
	std::cout << std::setw(4) << std::setfill('0');

	// Take the measurements!
	for (; !exitFlag && j < 100000; ++j) {
		std::cout << "\rReading" << j << " | ";
		for (int i = 0; !exitFlag && i < SENSOR_COUNT; ++i) {
			uint16_t distance;
			try {
				// Read the range. Note that it's a blocking call
				distance = sensors[i]->readRangeContinuousMillimeters();
			} catch (const std::exception & error) {
				std::cerr << std::endl << "Error geating measurement from sensor " << i << " with reason:" << std::endl << error.what() << std::endl;
				// You may want to bail out here, depending on your application - error means issues on I2C bus read/write.
				// return 3;
				distance = 8096;
			}

			// Check for timeout
			if (sensors[i]->timeoutOccurred()) {
				std::cout << "tout | ";
			} else {
				// Display the reading
				std::cout << distance << " | ";
			}
		}
		std::cout << std::endl << std::flush;

		// Calculate duration of current iteration
		std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
		uint64_t duration = (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1)).count();
		// Save current time as reference for next iteration
		t1 = t2;
		// Add total measurements duration
		totalDuration += duration;
		// Skip comparing first measurement against max and min as it's not a full iteration
		if (j == 0) {
			continue;
		}
		// Check and save max and min iteration duration
		if (duration > maxDuration) {
			maxDuration = duration;
		}
		if (duration < minDuration) {
			minDuration = duration;
		}
	}

	// Print measurement duration statistics
	std::cout << "\nMax duration: " << maxDuration << "ns" << std::endl;
	std::cout << "Min duration: " << minDuration << "ns" << std::endl;
	std::cout << "Avg duration: " << totalDuration/(j+1) << "ns" << std::endl;
	std::cout << "Avg frequency: " << 1000000000/(totalDuration/(j+1)) << "Hz" << std::endl;

	// Clean-up: delete objects, set GPIO/XSHUT pins to low.
	for (int i = 0; i < SENSOR_COUNT; ++i) {
		sensors[i]->stopContinuous();
		delete sensors[i];
	}
	return 0;
}

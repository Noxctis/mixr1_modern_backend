/** This example shows how to get single-shot range measurements from the VL53L0X.
 * Modified for maximum accuracy and long-range detection (up to 2000mm).
 */

#include "VL53L0X.hpp"
#include <chrono>
#include <csignal>
#include <exception>
#include <iomanip>
#include <iostream>
#include <unistd.h>

volatile sig_atomic_t exitFlag = 0;
void sigintHandler(int) {
    exitFlag = 1;
}

// ENABLED: Long range measurements (extends pulse periods)
#define LONG_RANGE
// ENABLED: High accuracy measurements (increases timing budget to 200ms)
#define HIGH_ACCURACY

#ifdef HIGH_SPEED
    #ifdef HIGH_ACCURACY
        #error HIGH_SPEED and HIGH_ACCURACY cannot be both enabled at once!
    #endif
#endif

// Piecewise calibration function to flatten residual non-linear error
uint16_t applyCalibrationOffset(uint16_t rawDistance) {
    if (rawDistance <= 500) {
        // Corrects the 30-40mm close-range overestimation
        return (rawDistance > 35) ? (rawDistance - 35) : 0; 
    } else {
        // Corrects the 10-20mm long-range overestimation
        return (rawDistance > 15) ? (rawDistance - 15) : 0;
    }
}

int main() {
    signal(SIGINT, sigintHandler);

    VL53L0X sensor;
    try {
        sensor.initialize();
        sensor.setTimeout(500); // Increased timeout to accommodate 200ms budget
    } catch (const std::exception & error) {
        std::cerr << "Error initializing sensor: " << error.what() << std::endl;
        return 1;
    }

    #ifdef LONG_RANGE
        try {
            sensor.setSignalRateLimit(0.1);
            sensor.setVcselPulsePeriod(VcselPeriodPreRange, 18);
            sensor.setVcselPulsePeriod(VcselPeriodFinalRange, 14);
        } catch (const std::exception & error) {
            std::cerr << "Error enabling long range mode: " << error.what() << std::endl;
            return 2;
        }
    #endif

    #ifdef HIGH_ACCURACY
        try {
            // Increase timing budget to 200 ms for maximum precision
            sensor.setMeasurementTimingBudget(200000);
        } catch (const std::exception & error) {
            std::cerr << "Error enabling high accuracy mode: " << error.what() << std::endl;
            return 3;
        }
    #endif

    if (exitFlag) return 0;

    uint64_t totalDuration = 0, maxDuration = 0, minDuration = 1000*1000*1000;
    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    int i = 0;
    std::cout << "\rReading" << std::setw(4) << std::setfill('0');
    
    for (; !exitFlag && i < 100000; ++i) {
        uint16_t rawDistance;
        try {
            rawDistance = sensor.readRangeSingleMillimeters();
        } catch (const std::exception & error) {
            std::cerr << "Error getting measurement: " << error.what() << std::endl;
            rawDistance = 8096;
        }

        if (sensor.timeoutOccurred()) {
            std::cout << "\rReading" << i << " | timeout!" << std::endl;
        } else {
            // Apply the custom software offset to the raw reading
            uint16_t calibratedDistance = applyCalibrationOffset(rawDistance);
            std::cout << "\rReading" << i << " | Raw: " << rawDistance << "mm | Calibrated: " << calibratedDistance << "mm" << std::endl;
        }
        std::cout << std::flush;

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
        uint64_t duration = (std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1)).count();
        t1 = t2;
        totalDuration += duration;
        
        if (i == 0) continue;
        if (duration > maxDuration) maxDuration = duration;
        if (duration < minDuration) minDuration = duration;
    }

    std::cout << "\nMax duration: " << maxDuration << "ns\nMin duration: " << minDuration << "ns\nAvg duration: " << totalDuration/(i+1) << "ns\nAvg frequency: " << 1000000000/(totalDuration/(i+1)) << "Hz\n";

    return 0;
}
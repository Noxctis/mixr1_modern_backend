#include "VL53L0X.hpp"
#include <wiringPi.h>
#include <iostream>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <vector>
#include <numeric>

// BCM Pin Definitions
#define MOTOR_INA 17
#define MOTOR_INB 27
#define MOTOR_PWM 13

volatile sig_atomic_t systemOffline = 0;
volatile sig_atomic_t emergencyStop = 0;

const char* CALIBRATION_FILE = "container_zero.txt";

// Hardware Interrupt Handler (Emergency Stop)
void sigintHandler(int) {
    emergencyStop = 1;
    systemOffline = 1;
}

// Failsafe Pump Shutdown
void stopPump() {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, LOW);
    pwmWrite(MOTOR_PWM, 0);
}

// Full Power Pump Activation
void runPump() {
    digitalWrite(MOTOR_INA, HIGH);
    digitalWrite(MOTOR_INB, LOW);
    pwmWrite(MOTOR_PWM, 1024);
}

// Fixed: Removed the breaking 21mm step-discontinuity.
// Standardizes a fixed linear offset if your sensor systematically over-reads.
uint16_t calibrateReading(uint16_t raw) {
    const uint16_t SENSOR_OFFSET = 35; // Adjust this fixed offset if needed
    if (raw > SENSOR_OFFSET) {
        return raw - SENSOR_OFFSET;
    }
    return 0;
}

// NEW: Helper function to collect a burst of readings and return a clean average
uint16_t getFilteredDistance(VL53L0X& sensor, int samples = 5) {
    std::vector<uint16_t> validReadings;
    
    for (int i = 0; i < samples; ++i) {
        try {
            uint16_t dist = sensor.readRangeSingleMillimeters();
            if (!sensor.timeoutOccurred() && dist > 0 && dist < 2000) {
                validReadings.push_back(dist);
            }
        } catch (...) {
            // Ignore single I2C dropouts during filtering burst
        }
        usleep(10000); // 10ms delay between filter bursts
    }

    if (validReadings.empty()) {
        return 0; // Signal an error state
    }

    // Calculate average of the collected burst
    long sum = std::accumulate(validReadings.begin(), validReadings.end(), 0);
    return static_cast<uint16_t>(sum / validReadings.size());
}

bool saveContainerZero(uint16_t containerZero) {
    std::ofstream file(CALIBRATION_FILE, std::ios::trunc);
    if (!file.is_open()) {
        return false;
    }
    file << containerZero;
    return static_cast<bool>(file);
}

uint16_t loadContainerZero() {
    std::ifstream file(CALIBRATION_FILE);
    uint16_t containerZero = 0;
    if (file.is_open()) {
        file >> containerZero;
    }
    return containerZero;
}

// Mode 1: Calibration
uint16_t runCalibration(VL53L0X& sensor) {
    std::cout << "\n--- [ MODE 1: CALIBRATION ] ---" << std::endl;
    std::cout << "Ensure the MIXR-1 container is EMPTY." << std::endl;
    std::cout << "Taking filtered measurements to establish baseline..." << std::endl;
    
    long totalDistance = 0;
    int validSamples = 0;
    
    for (int i = 0; i < 10; ++i) {
        if (emergencyStop) break;
        
        // Use the filter even during calibration for high accuracy
        uint16_t dist = getFilteredDistance(sensor, 5); 
        if (dist > 0) {
            dist = calibrateReading(dist);
            totalDistance += dist;
            validSamples++;
            std::cout << "Sampling " << i+1 << "/10 (Filtered): " << dist << " mm" << std::endl;
        } else {
            std::cerr << "[!] Warning: Dropped sample during calibration loop." << std::endl;
        }
        usleep(100000); // 100ms between step samples
    }
    
    if (validSamples == 0) {
        std::cout << "[!] Calibration failed. Check sensor wiring." << std::endl;
        return 0;
    }
    
    uint16_t containerZero = totalDistance / validSamples;
    std::cout << ">> Calibration Complete. Container bottom set at: " << containerZero << " mm.\n" << std::endl;

    if (saveContainerZero(containerZero)) {
        std::cout << "[SYSTEM] Calibration saved for next launch.\n" << std::endl;
    } else {
        std::cout << "[!] Warning: Calibration could not be saved.\n" << std::endl;
    }

    return containerZero;
}

// Mode 2: Fill Sequence
void runFillSequence(VL53L0X& sensor, uint16_t containerZero) {
    if (containerZero == 0) {
        std::cout << "\n[!] ERROR: You must run Calibration before starting a fill sequence.\n" << std::endl;
        return;
    }
    
    uint16_t targetLevel;
    std::cout << "\n--- [ MODE 2: START FILL ] ---" << std::endl;
    std::cout << "Container bottom is " << containerZero << " mm away." << std::endl;
    std::cout << "Enter target water level (height from bottom in mm): ";
    std::cin >> targetLevel;
    
    // Safety buffer check for ToF physical limits (approx 40mm physical dead zone from lens)
    if (targetLevel > containerZero) {
        std::cout << "[!] ERROR: Target water level cannot exceed container depth.\n" << std::endl;
        return;
    }

    std::cout << "\n[SYSTEM] Pump ACTIVATED." << std::endl;
    std::cout << ">>> PRESS [CTRL+C] FOR EMERGENCY STOP <<<\n" << std::endl;
    
    emergencyStop = 0; 
    runPump();
    
    int consecutiveErrors = 0;

    while (!emergencyStop) {
        // Collect 8 fast readings, drop noise, and return a clean average
        uint16_t dist = getFilteredDistance(sensor, 8);
        
        if (dist == 0) {
            consecutiveErrors++;
            if (consecutiveErrors > 5) { // 5 sequential failed loops triggers safety halt
                std::cerr << "\n[!] CRITICAL: Persistent sensor tracking lost! Emergency Stop Triggered." << std::endl;
                break;
            }
            continue; 
        }
        consecutiveErrors = 0; // Reset error counter on healthy reading
        
        dist = calibrateReading(dist);

        // Prevent mathematical underflow/overflow bounds
        int waterLevel = static_cast<int>(containerZero) - static_cast<int>(dist);
        if (waterLevel < 0) {
            waterLevel = 0;
        }

        std::cout << "\rCurrent Water Level: " << waterLevel << " mm from bottom | Target: "
                  << targetLevel << " mm        " << std::flush;
        
        // Bang-bang shutoff logic
        if (waterLevel >= targetLevel) {
            std::cout << "\n\n[SYSTEM] Target level reached. Pump SHUTTING DOWN.\n" << std::endl;
            break;
        }
        
        usleep(30000); // Small cooldown to keep I2C bus clean
    }
    
    stopPump();
    if (emergencyStop && !systemOffline) {
        std::cout << "\n[!] EMERGENCY STOP ENGAGED. Pump halted.\n" << std::endl;
    }
}

int main() {
    signal(SIGINT, sigintHandler);

    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Error: Failed to initialize WiringPi." << std::endl;
        return 1;
    }

    pinMode(MOTOR_INA, OUTPUT);
    pinMode(MOTOR_INB, OUTPUT);
    pinMode(MOTOR_PWM, PWM_OUTPUT);
    stopPump(); 

    VL53L0X sensor;
    try {
        sensor.initialize();
        sensor.setTimeout(500);
        // High accuracy mode setting retained
        sensor.setMeasurementTimingBudget(200000); 
    } catch (const std::exception & error) {
        std::cerr << "Error initializing ToF sensor: " << error.what() << std::endl;
        return 2;
    }

    uint16_t containerZero = loadContainerZero();
    int choice = 0;

    std::cout << "\n=========================================" << std::endl;
    std::cout << "     MIXR-1 FLUID CONTROL SYSTEM ONLINE   " << std::endl;
    std::cout << "=========================================" << std::endl;
    if (containerZero != 0) {
        std::cout << "Loaded saved container bottom: " << containerZero << " mm from sensor." << std::endl;
    }

    while (!systemOffline) {
        std::cout << "\nSelect an operation:" << std::endl;
        std::cout << "  [1] Calibrate Container Zero" << std::endl;
        std::cout << "  [2] Start Fill Sequence" << std::endl;
        std::cout << "  [3] Exit System / Emergency Stop" << std::endl;
        std::cout << "Selection: ";
        
        if (!(std::cin >> choice)) {
            break; 
        }

        switch (choice) {
            case 1:
                containerZero = runCalibration(sensor);
                break;
            case 2:
                runFillSequence(sensor, containerZero);
                std::cin.clear();
                break;
            case 3:
                systemOffline = 1;
                break;
            default:
                std::cout << "Invalid selection." << std::endl;
        }
    }

    stopPump();
    std::cout << "\nSystem Offline. Hardware safely parked." << std::endl;
    return 0;
}
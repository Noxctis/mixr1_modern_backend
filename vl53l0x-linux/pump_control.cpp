#include "VL53L0X.hpp"
#include <wiringPi.h>
#include <iostream>
#include <fstream>
#include <csignal>
#include <unistd.h>

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

// Piecewise calibration for non-linear sensor error
uint16_t calibrateReading(uint16_t raw) {
    if (raw <= 500 && raw > 35) return raw - 35;
    if (raw > 500 && raw > 15) return raw - 15;
    return raw;
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
    std::cout << "Taking 10 measurements to establish container baseline..." << std::endl;
    
    long totalDistance = 0;
    int validReadings = 0;
    
    for (int i = 0; i < 10; ++i) {
        if (emergencyStop) break;
        
        try {
            uint16_t dist = sensor.readRangeSingleMillimeters();
            if (!sensor.timeoutOccurred()) {
                dist = calibrateReading(dist);
                totalDistance += dist;
                validReadings++;
                std::cout << "Sampling " << i+1 << "/10: " << dist << " mm" << std::endl;
            }
        } catch (...) {
            std::cerr << "I2C read error during sampling." << std::endl;
        }
        usleep(250000); // Wait 250ms between samples
    }
    
    if (validReadings == 0) {
        std::cout << "[!] Calibration failed. Check sensor wiring." << std::endl;
        return 0;
    }
    
    uint16_t containerZero = totalDistance / validReadings;
    std::cout << ">> Calibration Complete. Container bottom set at: " << containerZero << " mm from sensor.\n" << std::endl;

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
    
    if (targetLevel > containerZero) {
        std::cout << "[!] ERROR: Target water level cannot be above the container bottom.\n" << std::endl;
        return;
    }

    std::cout << "\n[SYSTEM] Pump ACTIVATED." << std::endl;
    std::cout << ">>> PRESS [CTRL+C] FOR EMERGENCY STOP <<<\n" << std::endl;
    
    emergencyStop = 0; // Reset flag for loop
    runPump();
    
    while (!emergencyStop) {
        uint16_t dist;
        try {
            dist = sensor.readRangeSingleMillimeters();
        } catch (...) {
            std::cerr << "\n[!] CRITICAL: Sensor communication lost! Triggering Emergency Stop." << std::endl;
            break;
        }
        
        if (sensor.timeoutOccurred()) continue;
        
        dist = calibrateReading(dist);

        int waterLevel = static_cast<int>(containerZero) - static_cast<int>(dist);
        if (waterLevel < 0) {
            waterLevel = 0;
        }

        std::cout << "\rCurrent Water Level: " << waterLevel << " mm from bottom | Target: "
                  << targetLevel << " mm    " << std::flush;
        
        // Bang-bang shutoff logic
        if (waterLevel >= targetLevel) {
            std::cout << "\n\n[SYSTEM] Target level reached. Pump SHUTTING DOWN.\n" << std::endl;
            break;
        }
    }
    
    stopPump();
    if (emergencyStop && !systemOffline) {
        std::cout << "\n[!] EMERGENCY STOP ENGAGED. Pump halted.\n" << std::endl;
    }
}

int main() {
    // Register the hardware interrupt
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
        sensor.setMeasurementTimingBudget(200000); // Retain high accuracy mode
    } catch (const std::exception & error) {
        std::cerr << "Error initializing ToF sensor: " << error.what() << std::endl;
        return 2;
    }

    uint16_t containerZero = loadContainerZero();
    int choice = 0;

    std::cout << "\n=========================================" << std::endl;
    std::cout << "    MIXR-1 FLUID CONTROL SYSTEM ONLINE   " << std::endl;
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
            break; // Handle bad input or EOF
        }

        switch (choice) {
            case 1:
                containerZero = runCalibration(sensor);
                break;
            case 2:
                runFillSequence(sensor, containerZero);
                // Clear any residual cin errors after fill
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
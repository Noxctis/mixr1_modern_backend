// mixr1_fluid_controller.cpp
#include "VL53L0X.hpp"
#include <wiringPi.h>
#include <iostream>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <vector>
#include <numeric>
#include <termios.h>
#include <fcntl.h>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cmath>

// BCM Pin Definitions - PUMP (VNH5019 #1)
constexpr int PUMP_INA = 17;
constexpr int PUMP_INB = 27;
constexpr int PUMP_PWM = 13; // Hardware PWM1

// BCM Pin Definitions - SOLENOID (VNH5019 #2)
constexpr int SOLENOID_INA = 5;  // Reassigned from 23
constexpr int SOLENOID_INB = 6;  // Reassigned from 24
constexpr int SOLENOID_PWM = 12; // Hardware PWM0

volatile sig_atomic_t systemOffline = 0;
volatile sig_atomic_t emergencyStop = 0;

const char* CALIBRATION_FILE = "container_zero.txt";
const char* DATA_FILE = "fluid_dynamics_data.csv";

std::string currentSessionID;

struct SensorMetrics {
    uint16_t average;
    uint16_t min;
    uint16_t max;
    int validSamples;
    int targetSamples;
};

void setSolenoid(bool open) {
    if (open) {
        digitalWrite(SOLENOID_INA, HIGH);
        digitalWrite(SOLENOID_INB, LOW);
        pwmWrite(SOLENOID_PWM, 1024);
    } else {
        digitalWrite(SOLENOID_INA, LOW);
        digitalWrite(SOLENOID_INB, LOW);
        pwmWrite(SOLENOID_PWM, 0);
    }
}

void setPump(bool active) {
    if (active) {
        digitalWrite(PUMP_INA, HIGH);
        digitalWrite(PUMP_INB, LOW);
        pwmWrite(PUMP_PWM, 1024);
    } else {
        digitalWrite(PUMP_INA, LOW);
        digitalWrite(PUMP_INB, LOW);
        pwmWrite(PUMP_PWM, 0);
    }
}

void sigintHandler(int) {
    emergencyStop = 1;
    systemOffline = 1;
    setPump(false);
    setSolenoid(false); 
}

std::string generateSessionID() {
    std::time_t t = std::time(nullptr);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&t));
    return std::string(buffer);
}

int kbhit() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

    if (ch != EOF) {
        ungetc(ch, stdin);
        return 1;
    }
    return 0;
}

bool fileExists(const char* filename) {
    std::ifstream f(filename);
    return f.good();
}

bool saveCalibration(uint16_t containerZero, int floaterThickness) {
    std::ofstream file(CALIBRATION_FILE, std::ios::trunc);
    if (!file.is_open()) return false;
    file << containerZero << " " << floaterThickness;
    return static_cast<bool>(file);
}

bool loadCalibration(uint16_t& containerZero, int& floaterThickness) {
    std::ifstream file(CALIBRATION_FILE);
    if (file.is_open() && (file >> containerZero >> floaterThickness)) {
        return true;
    }
    containerZero = 0;
    floaterThickness = 0;
    return false;
}

SensorMetrics getSensorMetrics(VL53L0X& sensor, int samples, int delay_us = 10000) {
    std::vector<uint16_t> validReadings;
    SensorMetrics metrics = {0, 65535, 0, 0, samples};
    
    for (int i = 0; i < samples; ++i) {
        try {
            uint16_t dist = sensor.readRangeSingleMillimeters();
            if (!sensor.timeoutOccurred() && dist > 0 && dist < 2000) {
                validReadings.push_back(dist);
                if (dist < metrics.min) metrics.min = dist;
                if (dist > metrics.max) metrics.max = dist;
            }
        } catch (...) {}
        usleep(delay_us); 
    }

    metrics.validSamples = validReadings.size();
    if (metrics.validSamples == 0) {
        metrics.min = 0; 
        return metrics;
    }

    long sum = std::accumulate(validReadings.begin(), validReadings.end(), 0);
    metrics.average = static_cast<uint16_t>(sum / metrics.validSamples);
    
    return metrics;
}

void runCalibration(VL53L0X& sensor, uint16_t& containerZero, int& floaterThickness) {
    std::string dummy;
    std::cout << "\n--- [ MODE 1: DUAL CALIBRATION ] ---\n";
    std::cout << "[!] Ensure tank is completely EMPTY and FLOATER is REMOVED.\n";
    std::cout << "Press ENTER to measure tank bottom...";
    std::getline(std::cin, dummy);
    
    SensorMetrics bottomMetrics = getSensorMetrics(sensor, 20, 50000); 
    if (bottomMetrics.validSamples == 0) {
        std::cout << "[!] Calibration failed. Check sensor wiring.\n";
        return;
    }
    containerZero = bottomMetrics.average;
    
    std::cout << ">> Raw Container Bottom: " << containerZero << " mm\n";
    std::cout << "[!] Place FLOATER into the empty tank.\n";
    std::cout << "Press ENTER to measure floater thickness...";
    std::getline(std::cin, dummy);
    
    SensorMetrics floaterMetrics = getSensorMetrics(sensor, 20, 50000);
    if (floaterMetrics.validSamples == 0) {
        std::cout << "[!] Calibration failed. Resetting.\n";
        containerZero = 0;
        floaterThickness = 0;
        return;
    }
    
    floaterThickness = static_cast<int>(containerZero) - static_cast<int>(floaterMetrics.average);
    std::cout << ">> Floater Thickness: " << floaterThickness << " mm\n";
    saveCalibration(containerZero, floaterThickness);
}

void runSolenoidTestOnly() {
    std::string dummy;
    std::cout << "\n--- [ MODE 2: SOLENOID TOGGLE (DRY TEST) ] ---\n";
    std::cout << "Press ENTER to OPEN valve, ENTER again to CLOSE. Type 'q' and ENTER to quit.\n";
    
    bool isOpen = false;
    while (!emergencyStop) {
        std::getline(std::cin, dummy);
        if (dummy == "q") break;
        
        isOpen = !isOpen;
        setSolenoid(isOpen);
        std::cout << "[SYSTEM] Solenoid is now " << (isOpen ? "OPEN" : "CLOSED") << ".\n";
    }
    setSolenoid(false);
}

void runSolenoidAndToF(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }
    
    std::cout << "\n--- [ MODE 3: GRAVITY DRAIN & MONITOR ] ---\n";
    std::cout << "Opening solenoid and monitoring ToF drop. Press any key to abort.\n";
    
    while (kbhit()) getchar();
    emergencyStop = 0;
    setSolenoid(true);
    
    while (!emergencyStop && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 10000);
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            if (rawLevel < 0) rawLevel = 0;
            std::cout << "\rLvl: " << rawLevel << " mm | Yield: " << metrics.validSamples << "/3    " << std::flush;
            
            if (rawLevel <= 2) {
                std::cout << "\n[SYSTEM] Tank empty. Closing solenoid.\n";
                break;
            }
        }
    }
    
    if (kbhit()) getchar();
    setSolenoid(false);
}

void runFullFluidCycle(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }
    
    int targetLevel;
    std::cout << "\n--- [ MODE 4: FULL CYCLE (PUMP FILL -> SETTLE -> SOLENOID DRAIN) ] ---\n";
    std::cout << "Enter target fill level (mm): ";
    if (!(std::cin >> targetLevel)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        return;
    }
    std::cin.ignore(10000, '\n');

    emergencyStop = 0;
    setSolenoid(false);
    setPump(true);
    std::cout << "\n[PHASE 1] FILLING\n";
    
    while (!emergencyStop) {
        SensorMetrics metrics = getSensorMetrics(sensor, 5, 5000);
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::max(0, rawLevel) << "/" << targetLevel << " mm    " << std::flush;
            
            if (rawLevel >= targetLevel) break;
        }
    }
    
    setPump(false);
    if (emergencyStop) return;

    std::cout << "\n[PHASE 2] SETTLED. Press ENTER to open solenoid and drain...\n";
    std::string dummy;
    std::getline(std::cin, dummy);
    
    std::cout << "[PHASE 3] DRAINING\n";
    setSolenoid(true);
    
    while (!emergencyStop) {
        SensorMetrics metrics = getSensorMetrics(sensor, 5, 5000);
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::max(0, rawLevel) << " mm    " << std::flush;
            
            if (rawLevel <= 2) break;
        }
    }
    
    setSolenoid(false);
    std::cout << "\n[SYSTEM] Cycle complete. Hardware parked.\n";
}

void runContinuousRead(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0) return;
    std::cout << "\n--- [ MODE 5: CONTINUOUS SENSOR STREAM ] ---\nPress ANY KEY to stop.\n\n";
    while (kbhit()) getchar();

    while (!systemOffline && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 10000);
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::max(0, rawLevel) << " mm | Yield: " << metrics.validSamples << "/3    " << std::flush;
        }
    }
    if (kbhit()) getchar();
}

int main() {
    signal(SIGINT, sigintHandler);

    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Error: Failed to initialize WiringPi.\n";
        return 1;
    }

    pinMode(PUMP_INA, OUTPUT);
    pinMode(PUMP_INB, OUTPUT);
    pinMode(PUMP_PWM, PWM_OUTPUT);
    
    pinMode(SOLENOID_INA, OUTPUT);
    pinMode(SOLENOID_INB, OUTPUT);
    pinMode(SOLENOID_PWM, PWM_OUTPUT);
    
    setPump(false); 
    setSolenoid(false);

    currentSessionID = generateSessionID();
    VL53L0X sensor;
    try {
        sensor.initialize();
        sensor.setTimeout(500);
        sensor.setMeasurementTimingBudget(50000); 
    } catch (...) {
        std::cerr << "Error initializing ToF sensor.\n";
        return 2;
    }

    uint16_t containerZero = 0;
    int floaterThickness = 0;
    loadCalibration(containerZero, floaterThickness);

    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n=========================================\n";
        std::cout << " MIXR-1: FLUID DYNAMICS CONTROLLER\n";
        std::cout << "=========================================\n";
        std::cout << " [1] Set Container & Floater (Calibration)\n";
        std::cout << " [2] Solenoid Toggle (Dry Test)\n";
        std::cout << " [3] Gravity Drain & Monitor (Solenoid + ToF)\n";
        std::cout << " [4] Full Cycle (Pump Fill -> Settle -> Solenoid Drain)\n";
        std::cout << " [5] Continuous Sensor Stream\n";
        std::cout << " [6] Exit System\nSelection: ";
        
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            break; 
        }
        std::cin.ignore(10000, '\n'); 

        switch (choice) {
            case 1: runCalibration(sensor, containerZero, floaterThickness); break;
            case 2: runSolenoidTestOnly(); break;
            case 3: runSolenoidAndToF(sensor, containerZero, floaterThickness); break;
            case 4: runFullFluidCycle(sensor, containerZero, floaterThickness); break;
            case 5: runContinuousRead(sensor, containerZero, floaterThickness); break;
            case 6: systemOffline = 1; break;
        }
    }

    setPump(false);
    setSolenoid(false);
    std::cout << "\nSystem Offline. Hardware safely parked.\n";
    return 0;
}
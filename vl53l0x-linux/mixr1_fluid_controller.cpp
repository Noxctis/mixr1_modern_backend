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
#include <algorithm>

// BCM Pin Definitions
constexpr int PUMP_INA = 17;
constexpr int PUMP_INB = 27;
constexpr int PUMP_PWM = 13; 
constexpr int PUMP_EN  = 15; 

constexpr int SOLENOID_INA = 5;  
constexpr int SOLENOID_INB = 6;  
constexpr int SOLENOID_PWM = 12; 

volatile sig_atomic_t systemOffline = 0;
volatile sig_atomic_t emergencyStop = 0;

constexpr std::string_view CALIBRATION_FILE = "container_zero.txt";
constexpr std::string_view DATA_FILE = "fluid_dynamics_data.csv";
constexpr std::string_view RAW_DATA_FILE = "raw_sensor_data.csv";

std::string currentSessionID;

struct SensorMetrics {
    uint16_t average;
    uint16_t min;
    uint16_t max;
    int validSamples;
    int targetSamples;
};

void setSolenoid(bool open, int pwm_val = 1024) {
    if (open) {
        digitalWrite(SOLENOID_INA, HIGH);
        digitalWrite(SOLENOID_INB, LOW);
        pwmWrite(SOLENOID_PWM, pwm_val);
    } else {
        digitalWrite(SOLENOID_INA, LOW);
        digitalWrite(SOLENOID_INB, LOW);
        pwmWrite(SOLENOID_PWM, 0);
    }
}

void setPump(bool active, int pwm_val = 1024) {
    if (active) {
        digitalWrite(PUMP_EN, HIGH);
        digitalWrite(PUMP_INA, HIGH);
        digitalWrite(PUMP_INB, LOW);
        pwmWrite(PUMP_PWM, pwm_val);
    } else {
        digitalWrite(PUMP_EN, LOW);
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
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&t));
    return std::string(buffer);
}

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    std::time_t t = std::chrono::system_clock::to_time_t(now);
    char timeBuf[25];
    std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return std::string(timeBuf);
}

int kbhit() {
    struct timeval tv = {0L, 0L};
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(STDIN_FILENO, &fds);
    return select(1, &fds, nullptr, nullptr, &tv) > 0;
}

bool fileExists(std::string_view filename) {
    std::ifstream f(filename.data());
    return f.good();
}

bool saveCalibration(double containerZero, double floaterThickness) {
    std::ofstream file(CALIBRATION_FILE.data(), std::ios::trunc);
    if (!file.is_open()) return false;
    file << std::fixed << std::setprecision(2) << containerZero << " " << floaterThickness;
    return static_cast(file);
}

bool loadCalibration(double& containerZero, double& floaterThickness) {
    std::ifstream file(CALIBRATION_FILE.data());
    if (file.is_open() && (file >> containerZero >> floaterThickness)) {
        return true;
    }
    containerZero = 0.0;
    floaterThickness = 0.0;
    return false;
}

void logCycleData(const std::string& sessionID, int targetLevel, double calculatedLevel, double actualMeasured, uint16_t rawSensorAvg) {
    bool exists = fileExists(DATA_FILE);
    std::ofstream file(DATA_FILE.data(), std::ios::app);
    if (!file.is_open()) {
        std::cerr << "\n[!] Error opening CSV log file.\n";
        return;
    }

    if (!exists) {
        file << "SessionID,Timestamp,TargetLevel_mm,ToFCalculated_mm,ActualMeasured_mm,ToFRawAvg_mm,Error_mm\n";
    }

    double error = actualMeasured - calculatedLevel;
    file << sessionID << ","
         << getCurrentTimestamp() << ","
         << targetLevel << ","
         << std::fixed << std::setprecision(2) << calculatedLevel << ","
         << actualMeasured << ","
         << rawSensorAvg << ","
         << error << "\n";
}

void capture100Readings(VL53L0X& sensor, const std::string& eventName, double containerZero, double floaterThickness) {
    std::cout << "\n[DATA LOG] Fluid settling complete. Capturing 100 raw ToF readings...\n";
    bool exists = fileExists(RAW_DATA_FILE);
    std::ofstream rawFile(RAW_DATA_FILE.data(), std::ios::app);
    
    if (!exists) {
        rawFile << "SessionID,Timestamp,Event,SampleIndex,RawDistance_mm,CalculatedLevel_mm\n";
    }

    for (int i = 1; i <= 100; i++) {
        if (emergencyStop) break;
        uint16_t rawDist = 0;
        try {
            rawDist = sensor.readRangeSingleMillimeters();
        } catch (const std::exception& e) {
            std::cerr << "Sensor read error: " << e.what() << '\n';
        }

        double calculatedLevel = 0.0;
        if (containerZero > 0.0) {
            calculatedLevel = std::max(0.0, containerZero - (static_cast(rawDist) + floaterThickness)); 
        }

        std::cout << "\rSample " << i << "/100: Raw " << rawDist << " mm | Lvl " << std::fixed << std::setprecision(1) << calculatedLevel << " mm    " << std::flush;
        rawFile << currentSessionID << "," << getCurrentTimestamp() << "," << eventName << "," << i << "," << rawDist << "," << std::fixed << std::setprecision(2) << calculatedLevel << "\n";
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    std::cout << "\n[DATA LOG] Capture complete.\n";
}

SensorMetrics getSensorMetrics(VL53L0X& sensor, int samples, int delay_us = 10000) {
    std::vector validReadings;
    SensorMetrics metrics = {0, 65535, 0, 0, samples};
    
    for (int i = 0; i < samples; ++i) {
        if (emergencyStop) break; 
        
        try {
            uint16_t dist = sensor.readRangeSingleMillimeters();
            if (!sensor.timeoutOccurred() && dist > 0 && dist < 2000) {
                validReadings.push_back(dist);
                if (dist < metrics.min) metrics.min = dist;
                if (dist > metrics.max) metrics.max = dist;
            }
        } catch (const std::exception& e) {}
        std::this_thread::sleep_for(std::chrono::microseconds(delay_us)); 
    }

    metrics.validSamples = validReadings.size();
    if (metrics.validSamples == 0) {
        metrics.min = 0; 
        return metrics;
    }

    unsigned long sum = std::accumulate(validReadings.begin(), validReadings.end(), 0UL);
    metrics.average = static_cast(sum / static_cast(metrics.validSamples));
    
    return metrics;
}

int getPWMFromVoltage(const std::string& hwName) {
    double targetVoltage;
    std::cout << "Enter " << hwName << " operating voltage (6.0 - 12.0 V): ";
    if (!(std::cin >> targetVoltage) || targetVoltage < 6.0 || targetVoltage > 12.0) {
        std::cout << "[!] Invalid voltage. Defaulting to 12.0V.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n');
        return 1024;
    }
    std::cin.ignore(10000, '\n');
    
    int pwm_val = static_cast((targetVoltage / 12.0) * 1024.0);
    return std::clamp(pwm_val, 0, 1024);
}

// ==============================================================================
// HARDWARE CONTROL FUNCTIONS
// ==============================================================================

void runCalibration(VL53L0X& sensor, double& containerZero, double& floaterThickness) {
    std::string dummy;
    std::cout << "\n--- [ CALIBRATION: SET SYSTEM ZERO ] ---\n";
    std::cout << "[!] Ensure the tank is completely DRAINED.\n";
    std::cout << "[!] Place the FLOATER inside (let it rest naturally on the gasket).\n";
    std::cout << "Press ENTER to set system zero...";
    
    std::cin.clear();
    std::getline(std::cin, dummy);
    
    capture100Readings(sensor, "Calibration_SystemZero", 0.0, 0.0); 
    
    SensorMetrics zeroMetrics = getSensorMetrics(sensor, 20, 10000); 
    if (zeroMetrics.validSamples == 0) {
        std::cout << "[!] Calibration failed. Check sensor wiring.\n";
        return;
    }
    
    containerZero = static_cast(zeroMetrics.average);
    floaterThickness = 0.0; 
    
    std::cout << ">> System Zero (Distance to resting floater): " << std::fixed << std::setprecision(2) << containerZero << " mm\n";
    saveCalibration(containerZero, floaterThickness);
}

void runContinuousRead(VL53L0X& sensor, double containerZero, double floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }
    std::cout << "\n--- [ CONTINUOUS SENSOR STREAM ] ---\nPress ANY KEY to stop.\n\n";
    
    while (kbhit()) getchar(); 

    while (!systemOffline && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(sensor, 1, 0); 
        if (metrics.validSamples > 0) {
            double rawLevel = std::max(0.0, containerZero - (static_cast(metrics.average) + floaterThickness));
            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << rawLevel << " mm | Raw ToF: " << metrics.average << " mm    " << std::flush;
        }
    }
    
    if (kbhit()) getchar(); 
}

void runSolenoidTestOnly() {
    std::cout << "\n--- [ SOLENOID TOGGLE (DRY TEST) ] ---\n";
    int sol_pwm = getPWMFromVoltage("Solenoid");

    std::string dummy;
    std::cout << "Press ENTER to OPEN valve, ENTER again to CLOSE. Type 'q' and ENTER to quit.\n";
    
    bool isOpen = false;
    while (!emergencyStop) {
        std::getline(std::cin, dummy);
        if (dummy == "q" || dummy == "Q") break;
        
        isOpen = !isOpen;
        setSolenoid(isOpen, sol_pwm);
        std::cout << "[SYSTEM] Solenoid is now " << (isOpen ? "OPEN" : "CLOSED") << ".\n";
    }
    setSolenoid(false);
}

void runManualHardwareControl() {
    std::cout << "\n--- [ MANUAL HARDWARE CONTROL ] ---\n";
    std::cout << "Select Hardware:\n [1] Water Pump\n [2] Solenoid Valve\nSelection: ";
    
    int hwChoice;
    if (!(std::cin >> hwChoice) || (hwChoice != 1 && hwChoice != 2)) {
        std::cout << "[!] Invalid selection.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }

    std::string hwName = (hwChoice == 1) ? "Pump" : "Solenoid";
    int pwm_val = getPWMFromVoltage(hwName);

    std::cout << "\nSelect Control Mode:\n [1] Toggle\n [2] Timer\nSelection: ";
    int modeChoice;
    if (!(std::cin >> modeChoice) || (modeChoice != 1 && modeChoice != 2)) {
        std::cout << "[!] Invalid selection.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }
    std::cin.ignore(10000, '\n'); 

    if (modeChoice == 1) {
        std::cout << "\nPress ENTER to turn ON the " << hwName << "...";
        std::string dummy;
        std::getline(std::cin, dummy);

        if (hwChoice == 1) setPump(true, pwm_val);
        else setSolenoid(true, pwm_val);

        std::cout << "[" << hwName << " ON] Press ENTER to turn OFF...\n";
        std::getline(std::cin, dummy);

        if (hwChoice == 1) setPump(false);
        else setSolenoid(false);
        
        std::cout << "[" << hwName << " OFF]\n";
    } else {
        std::cout << "\nEnter duration in seconds: ";
        int seconds;
        if (!(std::cin >> seconds) || seconds <= 0) {
            std::cout << "[!] Invalid duration.\n";
            std::cin.clear(); std::cin.ignore(10000, '\n'); return;
        }
        std::cin.ignore(10000, '\n'); 

        if (hwChoice == 1) setPump(true, pwm_val);
        else setSolenoid(true, pwm_val);

        std::cout << "[" << hwName << " ON] Running for " << seconds << " seconds.\n";
        
        while (kbhit()) getchar();
        auto start_time = std::chrono::steady_clock::now();
        auto duration = std::chrono::seconds(seconds);

        while (std::chrono::steady_clock::now() - start_time < duration && !emergencyStop) {
            if (kbhit()) {
                getchar(); 
                std::cout << "\n[ABORTED] Manual interruption.\n";
                break;
            }
            auto elapsed = std::chrono::duration_cast(std::chrono::steady_clock::now() - start_time).count();
            std::cout << "\rRemaining: " << seconds - elapsed << " s   " << std::flush;
            std::this_thread::sleep_for(std::chrono::milliseconds(100)); 
        }

        if (hwChoice == 1) setPump(false);
        else setSolenoid(false);
        std::cout << "\n[" << hwName << " OFF]\n";
    }
}

// ==============================================================================
// EXPERIMENTAL TESTS
// ==============================================================================

void runTankZeroExperiment(VL53L0X& sensor) {
    std::cout << "\n--- [ TANK ZERO DISTANCE EXPERIMENT ] ---\n";
    double tankBottom = 0.0;
    int zeroChoice = 0;
    
    std::cout << "\nSet Tank Bottom Zero:\n [1] Auto-read via ToF\n [2] Manual distance\nSelection: ";
    if (!(std::cin >> zeroChoice) || (zeroChoice != 1 && zeroChoice != 2)) {
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }
    std::cin.ignore(10000, '\n');

    if (zeroChoice == 1) {
        std::cout << "\n[!] Ensure tank is EMPTY. Press ENTER to read...";
        std::string dummy; std::getline(std::cin, dummy);
        SensorMetrics bottomMetrics = getSensorMetrics(sensor, 20, 10000);
        if (bottomMetrics.validSamples == 0) return;
        tankBottom = static_cast(bottomMetrics.average);
    } else {
        std::cout << "\nEnter physical distance (mm): ";
        if (!(std::cin >> tankBottom) || tankBottom <= 0) return;
        std::cin.ignore(10000, '\n');
    }

    std::string dummy;
    std::cout << "[!] Place FLOATER in tank. Press ENTER to read resting distance...";
    std::getline(std::cin, dummy);

    SensorMetrics floaterMetrics = getSensorMetrics(sensor, 20, 10000);
    double floaterThickness = tankBottom - static_cast(floaterMetrics.average);

    int targetLevel, numIterations, numPhysicalMeasures;
    std::cout << "Enter target fluid level (mm): ";
    if (!(std::cin >> targetLevel)) return;
    std::cout << "Enter number of iterations: ";
    if (!(std::cin >> numIterations)) return;
    std::cout << "Enter physical measurements per iteration: ";
    if (!(std::cin >> numPhysicalMeasures)) return;
    std::cin.ignore(10000, '\n');

    int pump_pwm = getPWMFromVoltage("Pump");
    int sol_pwm = getPWMFromVoltage("Solenoid");

    bool exists = fileExists("tank_zero_experiment.csv");
    std::ofstream expFile("tank_zero_experiment.csv", std::ios::app);
    if (!exists) {
        expFile << "SessionID,Iteration,TargetLevel_mm,ToFAvg_mm,DistFromZero_mm,CalculatedFluid_mm,PhysicalIndex,PhysicalMeasure_mm\n";
    }

    emergencyStop = 0;
    bool abortExp = false; 

    for (int iter = 1; iter <= numIterations; iter++) {
        if (emergencyStop || abortExp) break;
        std::cout << "\n=== ITERATION " << iter << " / " << numIterations << " ===\n";

        std::cout << "[!] Ensure FLOATER is IN. Press ENTER to fill (or 'q' to abort)... ";
        std::getline(std::cin, dummy);
        if (dummy == "q" || dummy == "Q") break;

        while (kbhit()) getchar();
        setPump(true, pump_pwm);
        
        while (!emergencyStop) {
            if (kbhit()) { char c = getchar(); if (c == 'q' || c == 'Q') abortExp = true; break; }
            SensorMetrics metrics = getSensorMetrics(sensor, 3, 5000);
            if (metrics.validSamples > 0) {
                double rawLevel = std::max(0.0, tankBottom - (static_cast(metrics.average) + floaterThickness));
                std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << rawLevel << "/" << targetLevel << " mm    " << std::flush;
                if (rawLevel >= targetLevel) break;
            }
        }
        setPump(false);
        if (emergencyStop || abortExp) break;

        std::cout << "\n[SYSTEM] Settling fluid...\n";
        std::this_thread::sleep_for(std::chrono::seconds(2));

        std::cout << "Capturing ToF readings...\n";
        std::vector validReadings;
        for (int i = 1; i <= 100; i++) {
            if (emergencyStop) break;
            try {
                uint16_t dist = sensor.readRangeSingleMillimeters();
                if (!sensor.timeoutOccurred() && dist > 0 && dist < 2000) validReadings.push_back(dist);
            } catch (...) {}
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }

        double settledAvg = tankBottom;
        if (!validReadings.empty()) {
            settledAvg = static_cast(std::accumulate(validReadings.begin(), validReadings.end(), 0UL)) / validReadings.size();
        }

        double calcLevel = std::max(0.0, tankBottom - settledAvg - floaterThickness);
        std::cout << "\n>> Calculated Level: " << std::fixed << std::setprecision(2) << calcLevel << " mm\n\n";

        std::cout << "[!] REMOVE floater.\n";
        for (int p = 1; p <= numPhysicalMeasures; p++) {
            double pMeasure = 0.0;
            std::cout << "Physical measurement #" << p << " (mm) [-1 to EXIT]: ";
            if (!(std::cin >> pMeasure) || pMeasure < 0.0) { abortExp = true; break; }
            std::cin.ignore(10000, '\n');
            
            expFile << currentSessionID << "," << iter << "," << targetLevel << "," 
                    << std::fixed << std::setprecision(2) << settledAvg << "," << (tankBottom - settledAvg) << "," 
                    << calcLevel << "," << p << "," << pMeasure << "\n";
        }
        if (abortExp) break;

        std::cout << "[!] PLACE FLOATER IN. Press ENTER to drain (or 'q' to EXIT)... ";
        std::getline(std::cin, dummy);
        if (dummy == "q" || dummy == "Q") break;

        setSolenoid(true, sol_pwm);
        while (!emergencyStop) {
            SensorMetrics metrics = getSensorMetrics(sensor, 2, 10000);
            if (metrics.validSamples > 0) {
                double rawLevel = tankBottom - (static_cast(metrics.average) + floaterThickness);
                std::cout << "\rCurrent Lvl: " << std::fixed << std::setprecision(1) << std::max(0.0, rawLevel) << " mm    " << std::flush;
                if (rawLevel <= 2.0) break;
            }
        }
        setSolenoid(false);
    }
}

void runSolenoidAndToF(VL53L0X& sensor, double containerZero, double floaterThickness) {
    if (containerZero == 0.0) return;
    int sol_pwm = getPWMFromVoltage("Solenoid");
    std::cout << "Press ENTER to drain...\n";
    std::string dummy; std::getline(std::cin, dummy);
    
    setSolenoid(true, sol_pwm);
    while (!emergencyStop && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 10000);
        if (metrics.validSamples > 0) {
            double rawLevel = containerZero - (static_cast(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << std::max(0.0, rawLevel) << " mm    " << std::flush;
            if (rawLevel <= 2.0) break;
        }
    }
    setSolenoid(false);
}

void runFullFluidCycle(VL53L0X& sensor, double containerZero, double floaterThickness) {
    if (containerZero == 0.0) return;
    int pump_pwm = getPWMFromVoltage("Pump");
    int sol_pwm = getPWMFromVoltage("Solenoid");

    int targetLevel, drainInterval;
    std::cout << "Target fill level (mm): ";
    if (!(std::cin >> targetLevel)) return;
    std::cout << "Drain interval (mm): ";
    if (!(std::cin >> drainInterval)) return;
    std::cin.ignore(10000, '\n');

    std::cout << "\n[PHASE 1] FILLING. Press ENTER to start...";
    std::string dummy; std::getline(std::cin, dummy);

    setPump(true, pump_pwm);
    while (!emergencyStop) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 5000);
        if (metrics.validSamples > 0) {
            double rawLevel = containerZero - (static_cast(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << std::max(0.0, rawLevel) << "/" << targetLevel << " mm    " << std::flush;
            if (rawLevel >= targetLevel) break;
        }
    }
    setPump(false);

    std::this_thread::sleep_for(std::chrono::seconds(2));
    SensorMetrics settleMetrics = getSensorMetrics(sensor, 5, 10000);
    double calculatedLevel = std::max(0.0, containerZero - (static_cast(settleMetrics.average) + floaterThickness));

    std::cout << "\n[PHASE 2] SETTLED (ToF: " << calculatedLevel << " mm). Actual measure (mm): ";
    double actualMeasured = 0.0;
    if (std::cin >> actualMeasured) {
        logCycleData(currentSessionID, targetLevel, calculatedLevel, actualMeasured, settleMetrics.average);
    }
    std::cin.ignore(10000, '\n');

    std::cout << "\n[PHASE 3] DRAINING\n";
    while (!emergencyStop) {
        SensorMetrics currentMetrics = getSensorMetrics(sensor, 3, 5000);
        double currentLevel = std::max(0.0, containerZero - (static_cast(currentMetrics.average) + floaterThickness));
        if (currentLevel <= 2.0) break;

        std::cout << "\nPress ENTER to drain " << drainInterval << " mm...";
        std::getline(std::cin, dummy);
        
        double stepTarget = std::max(0.0, currentLevel - drainInterval);
        setSolenoid(true, sol_pwm);
        while (!emergencyStop) {
            SensorMetrics metrics = getSensorMetrics(sensor, 2, 5000);
            if (metrics.validSamples > 0) {
                double rawLevel = containerZero - (static_cast(metrics.average) + floaterThickness);
                std::cout << "\rCurrent: " << std::fixed << std::setprecision(1) << std::max(0.0, rawLevel) << " mm    " << std::flush;
                if (rawLevel <= stepTarget || rawLevel <= 2.0) break;
            }
        }
        setSolenoid(false);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}

// ==============================================================================
// MAIN ROUTINE
// ==============================================================================

int main() {
    signal(SIGINT, sigintHandler);

    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Error: Failed to initialize WiringPi.\n";
        return 1;
    }

    pinMode(PUMP_EN, OUTPUT);
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
        sensor.setMeasurementTimingBudget(200000); 
    } catch (...) {
        std::cerr << "Error initializing ToF sensor.\n";
        return 2;
    }

    double containerZero = 0.0, floaterThickness = 0.0;
    loadCalibration(containerZero, floaterThickness);

    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n=========================================\n"
                  << " MIXR-1: FLUID DYNAMICS CONTROLLER\n"
                  << "=========================================\n"
                  << " [1] Hardware Setup (Calibrate/Dry Test)\n"
                  << " [2] Tank Zero Distance Experiment\n"
                  << " [3] Gravity Drain & Monitor\n"
                  << " [4] Full Cycle (Fill & Drain)\n"
                  << " [5] Exit\nSelection: ";
        
        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); break; }
        std::cin.ignore(10000, '\n'); 

        switch (choice) {
            case 1: runCalibration(sensor, containerZero, floaterThickness); break;
            case 2: runTankZeroExperiment(sensor); break;
            case 3: runSolenoidAndToF(sensor, containerZero, floaterThickness); break;
            case 4: runFullFluidCycle(sensor, containerZero, floaterThickness); break;
            case 5: systemOffline = 1; break;
        }
    }

    setPump(false);
    setSolenoid(false);
    std::cout << "\nSystem Offline. Hardware safely parked.\n";
    return 0;
}
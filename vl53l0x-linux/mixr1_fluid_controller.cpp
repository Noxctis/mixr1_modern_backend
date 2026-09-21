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

// BCM Pin Definitions - PUMP (VNH5019 #1)
constexpr int PUMP_INA = 17;
constexpr int PUMP_INB = 27;
constexpr int PUMP_PWM = 13; // Hardware PWM1
constexpr int PUMP_EN  = 15; // BCM 15 - VNH5019 Enable

// BCM Pin Definitions - SOLENOID (VNH5019 #2)
constexpr int SOLENOID_INA = 5;  
constexpr int SOLENOID_INB = 6;  
constexpr int SOLENOID_PWM = 12; // Hardware PWM0

volatile sig_atomic_t systemOffline = 0;
volatile sig_atomic_t emergencyStop = 0;

const char* CALIBRATION_FILE = "container_zero.txt";
const char* DATA_FILE = "fluid_dynamics_data.csv";
const char* RAW_DATA_FILE = "raw_sensor_data.csv";

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

void logCycleData(const std::string& sessionID, int targetLevel, int calculatedLevel, double actualMeasured, uint16_t rawSensorAvg) {
    bool exists = fileExists(DATA_FILE);
    std::ofstream file(DATA_FILE, std::ios::app);
    if (!file.is_open()) {
        std::cerr << "\n[!] Error opening CSV log file.\n";
        return;
    }

    if (!exists) {
        file << "SessionID,Timestamp,TargetLevel_mm,ToFCalculated_mm,ActualMeasured_mm,ToFRawAvg_mm,Error_mm\n";
    }

    std::time_t t = std::time(nullptr);
    char timeBuf[20];
    std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

    double error = actualMeasured - calculatedLevel;

    file << sessionID << ","
         << timeBuf << ","
         << targetLevel << ","
         << calculatedLevel << ","
         << actualMeasured << ","
         << rawSensorAvg << ","
         << error << "\n";
}

void capture100Readings(VL53L0X& sensor, const std::string& eventName, uint16_t containerZero, int floaterThickness) {
    std::cout << "\n[DATA LOG] Fluid settling complete. Capturing 100 raw ToF readings (~20 seconds)...\n";
    bool exists = fileExists(RAW_DATA_FILE);
    std::ofstream rawFile(RAW_DATA_FILE, std::ios::app);
    
    if (!exists) {
        rawFile << "SessionID,Timestamp,Event,SampleIndex,RawDistance_mm,CalculatedLevel_mm\n";
    }

    for (int i = 1; i <= 100; i++) {
        if (emergencyStop) break;
        uint16_t rawDist = 0;
        try {
            // In High Accuracy mode (200ms budget), this call inherently blocks for ~200ms.
            rawDist = sensor.readRangeSingleMillimeters();
        } catch (...) {}

        // Calculate actual fluid level from the bottom
        int calculatedLevel = 0;
        if (containerZero > 0) {
            calculatedLevel = static_cast<int>(containerZero) - (static_cast<int>(rawDist) + floaterThickness);
            calculatedLevel = std::max(0, calculatedLevel); // Prevent negative outputs
        }

        std::time_t t = std::time(nullptr);
        char timeBuf[20];
        std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

        std::cout << "\rSample " << i << "/100: Raw " << rawDist << " mm | Lvl " << calculatedLevel << " mm    " << std::flush;
        rawFile << currentSessionID << "," << timeBuf << "," << eventName << "," << i << "," << rawDist << "," << calculatedLevel << "\n";
    }
    std::cout << "\n[DATA LOG] Capture complete.\n";
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

    unsigned long sum = std::accumulate(validReadings.begin(), validReadings.end(), 0UL);
    metrics.average = static_cast<uint16_t>(sum / static_cast<unsigned long>(metrics.validSamples));
    
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
    
    int pwm_val = static_cast<int>((targetVoltage / 12.0) * 1024.0);
    if (pwm_val > 1024) pwm_val = 1024;
    if (pwm_val < 0) pwm_val = 0;
    return pwm_val;
}

void runCalibration(VL53L0X& sensor, uint16_t& containerZero, int& floaterThickness) {
    std::string dummy;
    std::cout << "\n--- [ MODE 1: SINGLE-POINT RELATIVE CALIBRATION ] ---\n";
    std::cout << "[!] Ensure the tank is completely DRAINED.\n";
    std::cout << "[!] Place the FLOATER inside (let it rest naturally on the gasket).\n";
    std::cout << "Press ENTER to set system zero...";
    
    std::cin.clear();
    std::getline(std::cin, dummy);
    
    // Pass 0 for containerZero and floaterThickness so it only logs raw distance during calibration
    capture100Readings(sensor, "Calibration_SystemZero", 0, 0); 
    
    SensorMetrics zeroMetrics = getSensorMetrics(sensor, 20, 10000); 
    if (zeroMetrics.validSamples == 0) {
        std::cout << "[!] Calibration failed. Check sensor wiring.\n";
        return;
    }
    
    // The resting distance of the floater is our new absolute zero.
    containerZero = zeroMetrics.average;
    
    // We set thickness to 0 because the gasket height and floater thickness 
    // are now mathematically irrelevant to tracking the fluid level.
    floaterThickness = 0; 
    
    std::cout << ">> System Zero (Distance to resting floater): " << containerZero << " mm\n";
    saveCalibration(containerZero, floaterThickness);
}

void runContinuousRead(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }
    std::cout << "\n--- [ MODE 6: CONTINUOUS SENSOR STREAM ] ---\nPress ANY KEY to stop.\n\n";
    
    // Flush any pending terminal keys before starting the loop
    while (kbhit()) getchar(); 

    while (!systemOffline && !kbhit()) {
        // Reduced to 1 sample per loop. Because High Accuracy budget takes 200ms per sample, 
        // 1 sample ensures the terminal updates at 5 FPS instead of freezing for 600ms.
        SensorMetrics metrics = getSensorMetrics(sensor, 1, 0); 
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::max(0, rawLevel) << " mm | Raw ToF: " << metrics.average << " mm    " << std::flush;
        }
    }
    
    // Consume the key that stopped the loop so it doesn't bleed into the menu
    if (kbhit()) getchar(); 
}

void runSolenoidTestOnly() {
    std::cout << "\n--- [ MODE 2: SOLENOID TOGGLE (DRY TEST) ] ---\n";
    int sol_pwm = getPWMFromVoltage("Solenoid");

    std::string dummy;
    std::cout << "Press ENTER to OPEN valve, ENTER again to CLOSE. Type 'q' and ENTER to quit.\n";
    
    bool isOpen = false;
    while (!emergencyStop) {
        std::getline(std::cin, dummy);
        if (dummy == "q") break;
        
        isOpen = !isOpen;
        setSolenoid(isOpen, sol_pwm);
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
    int sol_pwm = getPWMFromVoltage("Solenoid");

    std::cout << "Opening solenoid and monitoring ToF drop. Press any key to abort.\n";
    
    while (kbhit()) getchar();
    emergencyStop = 0;
    setSolenoid(true, sol_pwm);
    
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
    
    std::cout << "\n--- [ MODE 4: FULL CYCLE (PUMP FILL -> SETTLE -> STEPPED DRAIN) ] ---\n";
    int pump_pwm = getPWMFromVoltage("Pump");
    int sol_pwm = getPWMFromVoltage("Solenoid");

    int targetLevel;
    std::cout << "Enter target fill level (mm): ";
    if (!(std::cin >> targetLevel)) {
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }

    int drainInterval;
    std::cout << "Enter drain step interval (mm): ";
    if (!(std::cin >> drainInterval) || drainInterval <= 0) {
        std::cout << "[!] Invalid drain interval.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }
    std::cin.ignore(10000, '\n');

    emergencyStop = 0;
    setSolenoid(false);
    setPump(true, pump_pwm);
    std::cout << "\n[PHASE 1] FILLING\n";
    
    while (!emergencyStop) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 5000);
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::max(0, rawLevel) << "/" << targetLevel << " mm    " << std::flush;
            if (rawLevel >= targetLevel) break;
        }
    }
    
    setPump(false);
    if (emergencyStop) return;

    // WAIT FOR FLUID TO SETTLE
    usleep(1000000); // 1 second settling time before capturing Data
    
    // CAPTURE RAW DATA WHILE SETTLED
    capture100Readings(sensor, "FullCycle_SettledMeasurement", containerZero, floaterThickness); 
    SensorMetrics settleMetrics = getSensorMetrics(sensor, 5, 10000);
    int calculatedLevel = static_cast<int>(containerZero) - (static_cast<int>(settleMetrics.average) + floaterThickness);

    std::cout << "\n\n[PHASE 2] SETTLED (ToF Calculated: " << std::max(0, calculatedLevel) << " mm)\n";
    std::cout << "Enter actual measured fluid level (mm): ";
    
    double actualMeasured = 0.0;
    if (std::cin >> actualMeasured) {
        std::cin.ignore(10000, '\n');
        logCycleData(currentSessionID, targetLevel, std::max(0, calculatedLevel), actualMeasured, settleMetrics.average);
    } else {
        std::cin.clear(); std::cin.ignore(10000, '\n');
    }

    std::cout << "\n[PHASE 3] STEPPED DRAINING (Step Size: " << drainInterval << " mm)\n";
    
    while (!emergencyStop) {
        SensorMetrics currentMetrics = getSensorMetrics(sensor, 3, 5000);
        int currentLevel = 0;
        if (currentMetrics.validSamples > 0) {
            currentLevel = std::max(0, static_cast<int>(containerZero) - (static_cast<int>(currentMetrics.average) + floaterThickness));
        }
        if (currentLevel <= 2) break;

        std::cout << "Current Level: " << currentLevel << " mm. Press ENTER to drain " << drainInterval << " mm (or 'q' to stop)... ";
        std::string input;
        std::getline(std::cin, input);
        if (input == "q" || input == "Q") break;
        
        int stepTarget = std::max(0, currentLevel - drainInterval);
        
        setSolenoid(true, sol_pwm);
        while (!emergencyStop) {
            SensorMetrics metrics = getSensorMetrics(sensor, 2, 5000);
            if (metrics.validSamples > 0) {
                int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
                std::cout << "\rDraining... Current: " << std::max(0, rawLevel) << " mm | Target: " << stepTarget << " mm    " << std::flush;
                if (rawLevel <= stepTarget || rawLevel <= 2) break;
            }
        }
        setSolenoid(false);
        std::cout << "\n[STEP COMPLETE] Solenoid closed.\n";
        
        // WAIT FOR FLUID TO SETTLE
        usleep(1000000); // 1 second settling time before capturing Data
        capture100Readings(sensor, "FullCycle_PostDrainStep", containerZero, floaterThickness);
        std::cout << "\n";
    }
    setSolenoid(false);
    std::cout << "\n[SYSTEM] Cycle complete. Hardware parked.\n";
}

void runExperimentDrainFlow(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }

    std::cout << "\n--- [ MODE 5: EXPERIMENT DRAIN FLOW (ITERATIVE) ] ---\n";
    int pump_pwm = getPWMFromVoltage("Pump");
    int sol_pwm = getPWMFromVoltage("Solenoid");

    int targetLevel;
    std::cout << "Enter initial fill set point (mm): ";
    if (!(std::cin >> targetLevel)) {
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }
    std::cin.ignore(10000, '\n');

    emergencyStop = 0;
    setSolenoid(false);
    setPump(true, pump_pwm);
    std::cout << "\n[PHASE 1] FILLING TO " << targetLevel << " mm\n";

    while (!emergencyStop) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 5000);
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            std::cout << "\rLvl: " << std::max(0, rawLevel) << "/" << targetLevel << " mm    " << std::flush;
            if (rawLevel >= targetLevel) break;
        }
    }
    setPump(false);
    if (emergencyStop) return;

    // WAIT FOR FLUID TO SETTLE
    usleep(1000000); 

    // CAPTURE RAW DATA WHILE SETTLED
    capture100Readings(sensor, "DrainExp_InitialMeasurement", containerZero, floaterThickness); 
    SensorMetrics settleMetrics = getSensorMetrics(sensor, 5, 10000);
    int calculatedLevel = static_cast<int>(containerZero) - (static_cast<int>(settleMetrics.average) + floaterThickness);

    std::cout << "\n\n[PHASE 2] MEASUREMENT\n";
    std::cout << "[!] REMOVE the floater and measure the water level manually.\n";
    std::cout << "Enter actual measured fluid level (mm): ";
    
    double actualMeasured = 0.0;
    if (std::cin >> actualMeasured) {
        std::cin.ignore(10000, '\n');
        logCycleData(currentSessionID, targetLevel, std::max(0, calculatedLevel), actualMeasured, settleMetrics.average);
    } else {
        std::cin.clear(); std::cin.ignore(10000, '\n');
    }

    std::cout << "\n[PHASE 3] ITERATIVE DRAIN\n";
    
    while (!emergencyStop) {
        int newSetPoint;
        std::cout << "\n[!] REPLACE the floater in the tank so the sensor can track the fluid.\n";
        std::cout << "Enter new lower set point to drain to (mm), or '0' to empty entirely: ";
        if (!(std::cin >> newSetPoint)) {
            std::cin.clear(); std::cin.ignore(10000, '\n'); break;
        }
        std::cin.ignore(10000, '\n');

        setSolenoid(true, sol_pwm);
        while (!emergencyStop) {
            SensorMetrics metrics = getSensorMetrics(sensor, 2, 5000);
            if (metrics.validSamples > 0) {
                int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
                std::cout << "\rDraining... Current Lvl: " << std::max(0, rawLevel) << " mm | Target: " << newSetPoint << " mm    " << std::flush;
                if (rawLevel <= newSetPoint || rawLevel <= 2) break;
            }
        }
        setSolenoid(false);
        if (emergencyStop) break;

        // WAIT FOR FLUID TO SETTLE
        usleep(1000000); 

        // CAPTURE RAW DATA WHILE SETTLED
        capture100Readings(sensor, "DrainExp_PostDrainMeasurement", containerZero, floaterThickness); 
        SensorMetrics drainSettleMetrics = getSensorMetrics(sensor, 5, 10000);
        int drainCalculated = static_cast<int>(containerZero) - (static_cast<int>(drainSettleMetrics.average) + floaterThickness);

        std::cout << "\n\n[!] DRAIN COMPLETE. REMOVE the floater again and measure manually.\n";
        std::cout << "Enter actual measured fluid level (mm): ";
        double drainMeasured = 0.0;
        if (std::cin >> drainMeasured) {
            std::cin.ignore(10000, '\n');
            logCycleData(currentSessionID, newSetPoint, std::max(0, drainCalculated), drainMeasured, drainSettleMetrics.average);
        } else {
            std::cin.clear(); std::cin.ignore(10000, '\n');
        }

        if (newSetPoint <= 2) {
            std::cout << "\n[SYSTEM] Tank reached bottom target.\n";
            break;
        }
    }
    
    setSolenoid(false);
    std::cout << "\n[SYSTEM] Experiment sequence complete. Hardware parked.\n";
}

void runManualHardwareControl() {
    std::cout << "\n--- [ MODE 7: MANUAL HARDWARE CONTROL ] ---\n";
    std::cout << "Select Hardware:\n";
    std::cout << " [1] Water Pump\n";
    std::cout << " [2] Solenoid Valve\n";
    std::cout << "Selection: ";
    
    int hwChoice;
    if (!(std::cin >> hwChoice) || (hwChoice != 1 && hwChoice != 2)) {
        std::cout << "[!] Invalid selection.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }

    std::string hwName = (hwChoice == 1) ? "Pump" : "Solenoid";
    int pwm_val = getPWMFromVoltage(hwName);

    std::cout << "\nSelect Control Mode:\n";
    std::cout << " [1] Toggle (Press Enter to start, Enter to stop)\n";
    std::cout << " [2] Timer (Run for X seconds)\n";
    std::cout << "Selection: ";
    
    int modeChoice;
    if (!(std::cin >> modeChoice) || (modeChoice != 1 && modeChoice != 2)) {
        std::cout << "[!] Invalid selection.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }
    std::cin.ignore(10000, '\n'); // Clear newline buffer

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
    } 
    else {
        std::cout << "\nEnter duration in seconds: ";
        int seconds;
        if (!(std::cin >> seconds) || seconds <= 0) {
            std::cout << "[!] Invalid duration.\n";
            std::cin.clear(); std::cin.ignore(10000, '\n'); return;
        }
        std::cin.ignore(10000, '\n'); // Clear newline buffer

        if (hwChoice == 1) setPump(true, pwm_val);
        else setSolenoid(true, pwm_val);

        std::cout << "[" << hwName << " ON] Running for " << seconds << " seconds. Press ANY KEY to abort.\n";
        
        // Flush any pending terminal keys before starting the loop
        while (kbhit()) getchar();

        auto start_time = std::time(nullptr);
        while (std::time(nullptr) - start_time < seconds && !emergencyStop) {
            if (kbhit()) {
                getchar(); // Consume the key
                std::cout << "\n[ABORTED] Manual interruption.\n";
                break;
            }
            std::cout << "\rRemaining: " << seconds - (std::time(nullptr) - start_time) << " s   " << std::flush;
            usleep(100000); // 100ms sleep to prevent CPU hogging
        }

        if (hwChoice == 1) setPump(false);
        else setSolenoid(false);
        
        std::cout << "\n[" << hwName << " OFF]\n";
    }
}

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
        
        // ==============================================================
        // HIGH ACCURACY PROFILE
        // Set timing budget to 200ms per measurement for highest accuracy 
        // ==============================================================
        sensor.setMeasurementTimingBudget(200000); 
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
        std::cout << " [4] Full Cycle (Pump Fill -> Settle -> Stepped Drain)\n";
        std::cout << " [5] Experiment Drain Flow (Fill -> Iterative Drain)\n";
        std::cout << " [6] Continuous Sensor Stream\n";
        std::cout << " [7] Manual Hardware Control (Toggle / Timer)\n";
        std::cout << " [8] Exit System\nSelection: ";
        
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
            case 5: runExperimentDrainFlow(sensor, containerZero, floaterThickness); break;
            case 6: runContinuousRead(sensor, containerZero, floaterThickness); break;
            case 7: runManualHardwareControl(); break;
            case 8: systemOffline = 1; break;
        }
    }

    setPump(false);
    setSolenoid(false);
    std::cout << "\nSystem Offline. Hardware safely parked.\n";
    return 0;
}
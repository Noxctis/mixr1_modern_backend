// mixr1_water_suite.cpp
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

// BCM Pin Definitions
#define MOTOR_INA 17
#define MOTOR_INB 27
#define MOTOR_PWM 13

volatile sig_atomic_t systemOffline = 0;
volatile sig_atomic_t emergencyStop = 0;

const char* CALIBRATION_FILE = "container_zero.txt";
const char* DATA_FILE = "water_surface_data.csv";

std::string currentSessionID;
int currentTestNumber = 1;

struct SensorMetrics {
    uint16_t average;
    uint16_t min;
    uint16_t max;
    int validSamples;
    int targetSamples;
};

void sigintHandler(int) {
    emergencyStop = 1;
    systemOffline = 1;
}

void stopPump() {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, LOW);
    pwmWrite(MOTOR_PWM, 0);
}

void runPump() {
    digitalWrite(MOTOR_INA, HIGH);
    digitalWrite(MOTOR_INB, LOW);
    pwmWrite(MOTOR_PWM, 1024);
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

void logToCSV(const std::string& sessionID, int testNumber, int trialNumber, int targetDistToWater, const SensorMetrics& metrics, int floaterThickness) {
    bool writeHeader = !fileExists(DATA_FILE);
    std::ofstream file(DATA_FILE, std::ios::app);
    
    int rawDistanceToWater = metrics.average + floaterThickness;
    int absoluteError = rawDistanceToWater - targetDistToWater;
    
    if (file.is_open()) {
        if (writeHeader) {
            file << "Session_ID,Test_Number,Trial_Number,Target_Dist_To_Water_mm,Raw_Dist_To_Floater_mm,Raw_Dist_To_Water_mm,Absolute_Error_mm,Min_Read_mm,Max_Read_mm,Valid_Samples,Target_Samples\n";
        }
        
        file << sessionID << "," 
             << testNumber << "," 
             << trialNumber << "," 
             << targetDistToWater << "," 
             << metrics.average << "," 
             << rawDistanceToWater << ","
             << absoluteError << ","
             << metrics.min << "," 
             << metrics.max << "," 
             << metrics.validSamples << "," 
             << metrics.targetSamples << "\n";
             
        std::cout << "[SYSTEM] Logged -> Raw Water Dist: " << rawDistanceToWater << " mm | Error: " 
                  << (absoluteError > 0 ? "+" : "") << absoluteError << " mm\n";
    } else {
        std::cerr << "[!] CRITICAL: Could not open " << DATA_FILE << " for writing.\n";
    }
}

void runCalibration(VL53L0X& sensor, uint16_t& containerZero, int& floaterThickness) {
    std::string dummy;
    std::cout << "\n--- [ MODE 1: DUAL CALIBRATION ] ---" << std::endl;
    
    // Step 1: Empty Tank Bottom
    std::cout << "\n[STEP 1/2] EMPTY TANK ZERO" << std::endl;
    std::cout << "[!] Ensure the MIXR-1 container is completely EMPTY." << std::endl;
    std::cout << "[!] Ensure the FLOATER IS REMOVED from the tank." << std::endl;
    std::cout << "Press ENTER to measure tank bottom...";
    std::getline(std::cin, dummy);
    
    SensorMetrics bottomMetrics = getSensorMetrics(sensor, 20, 50000); 
    if (bottomMetrics.validSamples == 0) {
        std::cout << "[!] Calibration failed on tank bottom. Check sensor wiring." << std::endl;
        return;
    }
    containerZero = bottomMetrics.average;
    
    // Step 2: Floater Thickness
    std::cout << "\n[STEP 2/2] FLOATER THICKNESS" << std::endl;
    std::cout << ">> Raw Container Bottom: " << containerZero << " mm" << std::endl;
    std::cout << "[!] Now place the FLOATER into the empty tank (resting on the bottom)." << std::endl;
    std::cout << "Press ENTER to measure floater thickness...";
    std::getline(std::cin, dummy);
    
    SensorMetrics floaterMetrics = getSensorMetrics(sensor, 20, 50000);
    if (floaterMetrics.validSamples == 0) {
        std::cout << "[!] Calibration failed on floater. Resetting calibration." << std::endl;
        containerZero = 0;
        floaterThickness = 0;
        return;
    }
    
    uint16_t floaterZero = floaterMetrics.average;
    floaterThickness = static_cast<int>(containerZero) - static_cast<int>(floaterZero);

    std::cout << "\n>> Calibration Complete." << std::endl;
    std::cout << ">> Container Bottom: " << containerZero << " mm" << std::endl;
    std::cout << ">> Floater Thickness: " << floaterThickness << " mm" << std::endl;

    if (saveCalibration(containerZero, floaterThickness)) {
        std::cout << "[SYSTEM] Calibration saved for next launch.\n" << std::endl;
    } else {
        std::cout << "[!] Warning: Calibration could not be saved.\n" << std::endl;
    }
}

void runFillSequence(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0 || floaterThickness == 0) {
        std::cout << "\n[!] ERROR: You must run Dual Calibration before starting a fill sequence.\n" << std::endl;
        return;
    }
    
    std::cout << "\n--- [ MODE 2: START PUMP FILL (NO LOGGING) ] ---" << std::endl;
    
    uint16_t targetLevel;
    std::cout << "Enter target water level (height from bottom in mm): ";
    std::cin >> targetLevel;
    
    if (targetLevel > (containerZero - floaterThickness)) {
        std::cout << "[!] ERROR: Target exceeds safe fill height. Floater will collide with sensor.\n" << std::endl;
        return;
    }

    std::cout << "\n[SYSTEM] Pump ACTIVATED." << std::endl;
    std::cout << ">>> PRESS [CTRL+C] FOR EMERGENCY STOP <<<\n" << std::endl;
    
    emergencyStop = 0; 
    int consecutiveLosses = 0;
    
    runPump();
    
    while (!emergencyStop) {
        SensorMetrics metrics = getSensorMetrics(sensor, 5, 5000); 
        
        if (metrics.validSamples == 0) {
            consecutiveLosses++;
            if (consecutiveLosses >= 6) {
                std::cerr << "\n\n[!] CRITICAL: Complete IR signal loss! Emergency Halt." << std::endl;
                break;
            }
            continue; 
        }
        consecutiveLosses = 0; 

        int rawWaterLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
        if (rawWaterLevel < 0) rawWaterLevel = 0;

        std::cout << "\rLvl: " << rawWaterLevel << "/" << targetLevel << " mm | Yield: " << metrics.validSamples << "/5    " << std::flush;
        
        if (rawWaterLevel >= targetLevel) {
            std::cout << "\n\n[SYSTEM] Target level reached. Pump SHUTTING DOWN.\n" << std::endl;
            break;
        }
    }
    
    stopPump();
    if (emergencyStop && !systemOffline) {
        std::cout << "\n[!] EMERGENCY STOP ENGAGED. Pump halted.\n" << std::endl;
    }
}

void runAutoFillAndRecord(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0 || floaterThickness == 0) {
        std::cout << "\n[!] ERROR: You must run Dual Calibration (Mode 1) first.\n" << std::endl;
        return;
    }

    std::cout << "\n--- [ MODE 3: AUTO FILL & RECORD LOOP ] ---" << std::endl;
    
    int targetLevel;
    std::cout << "Enter target water level from bottom (mm): ";
    if (!(std::cin >> targetLevel)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "[!] Invalid input. Returning to menu." << std::endl;
        return;
    }

    if (targetLevel > (containerZero - floaterThickness)) {
        std::cout << "[!] ERROR: Target water level exceeds safe container depth." << std::endl;
        return;
    }

    int expectedDistanceToWater = containerZero - targetLevel;

    int trials;
    std::cout << "Enter number of pump-and-record trials (e.g., 5): ";
    if (!(std::cin >> trials)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "[!] Invalid input. Returning to menu." << std::endl;
        return;
    }
    std::cin.ignore(10000, '\n'); // Clear buffer for upcoming getlines

    std::cout << "\n[SYSTEM] Using Container Zero: " << containerZero << " mm" << std::endl;
    std::cout << "[SYSTEM] Expected sensor reading to WATER surface: " << expectedDistanceToWater << " mm" << std::endl;

    for (int t = 1; t <= trials; ++t) {
        std::cout << "\n========================================" << std::endl;
        std::cout << " TRIAL " << t << " OF " << trials << std::endl;
        std::cout << "========================================" << std::endl;

        std::cout << "\n[PHASE 1] PUMPING TO " << targetLevel << " mm" << std::endl;
        std::cout << "Ensure tank is drained to your starting level, then press ENTER to activate pump...";
        std::string dummy;
        std::getline(std::cin, dummy);

        emergencyStop = 0; 
        int consecutiveLosses = 0;
        runPump();
        
        while (!emergencyStop) {
            SensorMetrics metrics = getSensorMetrics(sensor, 5, 5000); 
            
            if (metrics.validSamples == 0) {
                consecutiveLosses++;
                if (consecutiveLosses >= 6) {
                    std::cerr << "\n\n[!] CRITICAL: Complete IR signal loss! Emergency Halt." << std::endl;
                    break;
                }
                continue; 
            }
            consecutiveLosses = 0; 

            int rawWaterLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            if (rawWaterLevel < 0) rawWaterLevel = 0;

            std::cout << "\rLvl: " << rawWaterLevel << "/" << targetLevel << " mm | Yield: " << metrics.validSamples << "/5    " << std::flush;
            
            if (rawWaterLevel >= targetLevel) {
                std::cout << "\n\n[SYSTEM] Target level reached. Pump SHUTTING DOWN.\n" << std::endl;
                break;
            }
        }
        
        stopPump();
        if (emergencyStop && !systemOffline) {
            std::cout << "\n[!] EMERGENCY STOP ENGAGED. Canceling remaining trials.\n" << std::endl;
            return;
        }

        std::cout << "\n[PHASE 2] SENSOR RECORDING" << std::endl;
        std::cout << "Wait for the water to settle completely, then press ENTER to record data...";
        std::getline(std::cin, dummy);

        SensorMetrics metrics = getSensorMetrics(sensor, 10);

        if (metrics.validSamples == 0) {
            std::cout << "[!] ERROR: Failed to get any valid readings. Check floater placement." << std::endl;
            t--; // Retry this trial loop if recording failed
            continue;
        }

        logToCSV(currentSessionID, currentTestNumber, t, expectedDistanceToWater, metrics, floaterThickness);
    }
    
    currentTestNumber++; 
    std::cout << "\n>> Finished all " << trials << " pump-and-record trials." << std::endl;
}

void runContinuousRead(VL53L0X& sensor, uint16_t containerZero, int floaterThickness) {
    if (containerZero == 0 || floaterThickness == 0) {
        std::cout << "\n[!] ERROR: You must run Dual Calibration (Mode 1) first.\n" << std::endl;
        return;
    }

    std::cout << "\n--- [ MODE 4: CONTINUOUS STREAM ] ---" << std::endl;
    std::cout << "Streaming actual water level (accounting for " << floaterThickness << "mm dynamic floater thickness).\n";
    std::cout << "Press ANY KEY to stop.\n\n";

    while (kbhit()) getchar();

    while (!systemOffline && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 10000);
        
        if (metrics.validSamples > 0) {
            int rawLevel = static_cast<int>(containerZero) - (static_cast<int>(metrics.average) + floaterThickness);
            if (rawLevel < 0) rawLevel = 0;

            std::cout << "\rLvl: " << rawLevel << " mm (" << (metrics.average + floaterThickness) << " dist) | "
                      << "Yield: " << metrics.validSamples << "/3       " << std::flush;
        } else {
            std::cout << "\r[SCATTERED/LOST SENSOR]                                                                 " << std::flush;
        }
    }

    if (kbhit()) getchar();
    std::cout << "\n\nStopped continuous read." << std::endl;
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

    currentSessionID = generateSessionID();

    VL53L0X sensor;
    try {
        sensor.initialize();
        sensor.setTimeout(500);
        sensor.setMeasurementTimingBudget(50000); 
    } catch (const std::exception & error) {
        std::cerr << "Error initializing ToF sensor: " << error.what() << std::endl;
        return 2;
    }

    uint16_t containerZero = 0;
    int floaterThickness = 0;
    loadCalibration(containerZero, floaterThickness);

    int choice = 0;

    std::cout << "\n=========================================" << std::endl;
    std::cout << "  MIXR-1: MASTER WATER TESTING SUITE     " << std::endl;
    std::cout << "  Session ID: " << currentSessionID << std::endl;
    std::cout << "=========================================" << std::endl;
    
    if (containerZero != 0 && floaterThickness != 0) {
        std::cout << "Loaded saved calibration:\n"
                  << "  Container Zero:    " << containerZero << " mm\n"
                  << "  Floater Thickness: " << floaterThickness << " mm\n";
    }

    while (!systemOffline) {
        std::cout << "\nSelect an operation:" << std::endl;
        std::cout << "  [1] Set Container & Floater (Dual Calibration)" << std::endl;
        std::cout << "  [2] Start Pump Fill Sequence (No Logging)" << std::endl;
        std::cout << "  [3] Auto-Fill & Record Data (Pump + ToF Loop)" << std::endl;
        std::cout << "  [4] Continuous Sensor Stream" << std::endl;
        std::cout << "  [5] Exit System" << std::endl;
        std::cout << "Selection: ";
        
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            break; 
        }
        std::cin.ignore(10000, '\n'); // Clear the newline from the buffer

        switch (choice) {
            case 1:
                runCalibration(sensor, containerZero, floaterThickness);
                break;
            case 2:
                runFillSequence(sensor, containerZero, floaterThickness);
                break;
            case 3:
                runAutoFillAndRecord(sensor, containerZero, floaterThickness);
                break;
            case 4:
                runContinuousRead(sensor, containerZero, floaterThickness);
                break;
            case 5:
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
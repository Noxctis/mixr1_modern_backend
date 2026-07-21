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

// Session Variables
std::string currentSessionID;
int currentTestNumber = 1;

// Data Structure to hold comprehensive burst metrics
struct SensorMetrics {
    uint16_t average;
    uint16_t min;
    uint16_t max;
    int validSamples;
    int targetSamples;
};

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
    pwmWrite(MOTOR_PWM, 256);
}

// Generates a unique timestamp string for the session ID
std::string generateSessionID() {
    std::time_t t = std::time(nullptr);
    char buffer[20];
    std::strftime(buffer, sizeof(buffer), "%Y%m%d_%H%M%S", std::localtime(&t));
    return std::string(buffer);
}

// Non-blocking keyboard check for Linux to exit continuous loops cleanly
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

bool saveContainerZero(uint16_t containerZero) {
    std::ofstream file(CALIBRATION_FILE, std::ios::trunc);
    if (!file.is_open()) return false;
    file << containerZero;
    return static_cast<bool>(file);
}

uint16_t loadContainerZero() {
    std::ifstream file(CALIBRATION_FILE);
    uint16_t containerZero = 0;
    if (file.is_open()) file >> containerZero;
    return containerZero;
}

// Core filtering engine: Collects a burst of readings and extracts precision metrics
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
        } catch (...) {
            // Ignore I2C dropouts during burst
        }
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

// Restored 12-Column CSV Logging with Calibration Formula
void logToCSV(const std::string& sessionID, int testNumber, int trialNumber, int targetDistance, const SensorMetrics& metrics) {
    bool writeHeader = !fileExists(DATA_FILE);
    std::ofstream file(DATA_FILE, std::ios::app);
    
    int absoluteError = static_cast<int>(metrics.average) - targetDistance;
    float calibratedReading = (static_cast<float>(metrics.average) - 9.067f) / 1.052f;
    float calibratedError = calibratedReading - static_cast<float>(targetDistance);
    
    if (file.is_open()) {
        if (writeHeader) {
            file << "Session_ID,Test_Number,Trial_Number,Target_Distance_mm,Sensor_Avg_mm,Absolute_Error_mm,Min_Read_mm,Max_Read_mm,Valid_Samples,Target_Samples,Calibrated_Reading_mm,Calibrated_Error_mm\n";
        }
        
        file << sessionID << "," 
             << testNumber << "," 
             << trialNumber << "," 
             << targetDistance << "," 
             << metrics.average << "," 
             << absoluteError << ","
             << metrics.min << "," 
             << metrics.max << "," 
             << metrics.validSamples << "," 
             << metrics.targetSamples << ","
             << std::fixed << std::setprecision(2) << calibratedReading << ","
             << std::fixed << std::setprecision(2) << calibratedError << "\n";
             
        std::cout << "[SYSTEM] Logged -> Raw: " << metrics.average << " mm | Calib: " 
                  << std::fixed << std::setprecision(2) << calibratedReading << " mm | Calib Error: " 
                  << (calibratedError > 0 ? "+" : "") << calibratedError << " mm\n";
                  
    } else {
        std::cerr << "[!] CRITICAL: Could not open " << DATA_FILE << " for writing.\n";
    }
}

// MODE 1: Calibrate Container Bottom
uint16_t runCalibration(VL53L0X& sensor) {
    std::cout << "\n--- [ MODE 1: CONTAINER ZERO ] ---" << std::endl;
    std::cout << "Ensure the MIXR-1 container is EMPTY." << std::endl;
    std::cout << "Taking high-fidelity burst measurement to establish baseline..." << std::endl;
    
    SensorMetrics metrics = getSensorMetrics(sensor, 20, 50000); 

    if (metrics.validSamples == 0) {
        std::cout << "[!] Calibration failed. Check sensor wiring or target distance." << std::endl;
        return 0;
    }
    
    uint16_t containerZero = metrics.average;
    float calibratedZero = (static_cast<float>(containerZero) - 9.067f) / 1.052f;

    std::cout << ">> Calibration Complete." << std::endl;
    std::cout << ">> Raw Container Bottom: " << containerZero << " mm" << std::endl;
    std::cout << ">> Calibrated Bottom (Aligns with Ruler): " << std::fixed << std::setprecision(2) << calibratedZero << " mm\n" << std::defaultfloat << std::endl;

    if (saveContainerZero(containerZero)) {
        std::cout << "[SYSTEM] Calibration saved for next launch.\n" << std::endl;
    } else {
        std::cout << "[!] Warning: Calibration could not be saved.\n" << std::endl;
    }

    return containerZero;
}

// MODE 2: Fluid Fill Sequence (Selectable Logic)
void runFillSequence(VL53L0X& sensor, uint16_t containerZero) {
    if (containerZero == 0) {
        std::cout << "\n[!] ERROR: You must run Calibration before starting a fill sequence.\n" << std::endl;
        return;
    }
    
    float calibratedContainerZero = (static_cast<float>(containerZero) - 9.067f) / 1.052f;
    int controlChoice = 0;
    
    std::cout << "\n--- [ MODE 2: START PUMP FILL ] ---" << std::endl;
    std::cout << "Select Pump Control Logic:" << std::endl;
    std::cout << "  [1] RAW Logic        (Container Zero: " << containerZero << " mm)" << std::endl;
    std::cout << "  [2] CALIBRATED Logic (Container Zero: " << std::fixed << std::setprecision(2) << calibratedContainerZero << " mm)" << std::defaultfloat << std::endl;
    std::cout << "Selection: ";
    
    if (!(std::cin >> controlChoice) || (controlChoice != 1 && controlChoice != 2)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "[!] Invalid logic selection. Returning to menu.\n" << std::endl;
        return;
    }
    
    uint16_t targetLevel;
    std::cout << "Enter target water level (height from bottom in mm): ";
    std::cin >> targetLevel;
    
    if (controlChoice == 1 && targetLevel > containerZero) {
        std::cout << "[!] ERROR: Target exceeds raw container depth.\n" << std::endl;
        return;
    } else if (controlChoice == 2 && targetLevel > calibratedContainerZero) {
        std::cout << "[!] ERROR: Target exceeds calibrated container depth.\n" << std::endl;
        return;
    }

    std::cout << "\n[SYSTEM] Pump ACTIVATED using " << (controlChoice == 1 ? "RAW" : "CALIBRATED") << " logic." << std::endl;
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

        // Calculate both logic paths concurrently
        float calibratedDist = (static_cast<float>(metrics.average) - 9.067f) / 1.052f;
        
        int rawWaterLevel = static_cast<int>(containerZero) - static_cast<int>(metrics.average);
        float calibWaterLevel = calibratedContainerZero - calibratedDist;
        
        if (rawWaterLevel < 0) rawWaterLevel = 0;
        if (calibWaterLevel < 0) calibWaterLevel = 0;

        // Assign the active level driving the pump shutoff based on user selection
        float activeWaterLevel = (controlChoice == 1) ? static_cast<float>(rawWaterLevel) : calibWaterLevel;

        std::cout << "\rLevel: " << std::fixed << std::setprecision(1) << activeWaterLevel << "/" << targetLevel << " mm | Raw: " 
                  << metrics.average << " mm | Calib: " << calibratedDist << " mm | Yield: " << metrics.validSamples << "/5    " << std::flush;
        
        if (activeWaterLevel >= targetLevel) {
            std::cout << "\n\n[SYSTEM] Target level reached. Pump SHUTTING DOWN.\n" << std::endl;
            break;
        }
    }
    
    std::cout << std::defaultfloat; // Reset formatting
    stopPump();
    if (emergencyStop && !systemOffline) {
        std::cout << "\n[!] EMERGENCY STOP ENGAGED. Pump halted.\n" << std::endl;
    }
}

// MODE 3: Record Static Water Data
void runRecordDataPoint(VL53L0X& sensor, uint16_t containerZero) {
    if (containerZero == 0) {
        std::cout << "\n[!] ERROR: You must run Calibration (Mode 1) first.\n" << std::endl;
        return;
    }

    float calibratedContainerZero = (static_cast<float>(containerZero) - 9.067f) / 1.052f;

    std::cout << "\n--- [ MODE 3: DATA RECORDING ] ---" << std::endl;
    
    int targetLevel;
    std::cout << "Enter the physical water level from bottom (mm): ";
    if (!(std::cin >> targetLevel)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "[!] Invalid input. Returning to menu." << std::endl;
        return;
    }

    if (targetLevel > calibratedContainerZero) {
        std::cout << "[!] ERROR: Target water level exceeds known container depth." << std::endl;
        return;
    }

    // Calculates the true expected distance from the sensor to the water surface using the calibrated geometry
    int expectedDistance = static_cast<int>(std::round(calibratedContainerZero - targetLevel));

    int trials;
    std::cout << "Enter number of trials for this depth (e.g., 5): ";
    if (!(std::cin >> trials)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "[!] Invalid input. Returning to menu." << std::endl;
        return;
    }
    
    std::cin.ignore(10000, '\n');

    std::cout << "\n[SYSTEM] Using Calibrated Container Zero: " << std::fixed << std::setprecision(2) << calibratedContainerZero << " mm" << std::defaultfloat << std::endl;
    std::cout << "[SYSTEM] Desired Water Level: " << targetLevel << " mm" << std::endl;
    std::cout << "[SYSTEM] Expected sensor reading to surface: " << expectedDistance << " mm" << std::endl;

    for (int t = 1; t <= trials; ++t) {
        std::cout << "\n[Trial " << t << "/" << trials << "] Wait for water to settle, then press ENTER...";
        std::string dummy;
        std::getline(std::cin, dummy);

        SensorMetrics metrics = getSensorMetrics(sensor, 10);

        if (metrics.validSamples == 0) {
            std::cout << "[!] ERROR: Failed to get any valid readings." << std::endl;
            t--; 
            continue;
        }

        // Pass expectedDistance so the CSV columns properly calculate errors against the calibrated ruler geometry
        logToCSV(currentSessionID, currentTestNumber, t, expectedDistance, metrics);
    }
    
    currentTestNumber++; 
    std::cout << "\n>> Finished all " << trials << " trials." << std::endl;
}

// MODE 4: Continuous Read Stream
void runContinuousRead(VL53L0X& sensor, uint16_t containerZero) {
    if (containerZero == 0) {
        std::cout << "\n[!] ERROR: You must run Calibration (Mode 1) first.\n" << std::endl;
        return;
    }

    float calibratedContainerZero = (static_cast<float>(containerZero) - 9.067f) / 1.052f;

    std::cout << "\n--- [ MODE 4: CONTINUOUS STREAM ] ---" << std::endl;
    std::cout << "Streaming sensor data. Press ANY KEY to stop.\n\n";

    while (kbhit()) getchar();
    std::cout << std::fixed << std::setprecision(1);

    while (!systemOffline && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 10000);
        
        if (metrics.validSamples > 0) {
            float calibratedDist = (static_cast<float>(metrics.average) - 9.067f) / 1.052f;
            
            // Calculate Water Levels
            int rawLevel = static_cast<int>(containerZero) - static_cast<int>(metrics.average);
            float calibLevel = calibratedContainerZero - calibratedDist;
            
            std::cout << "\rRaw Lvl: " << rawLevel << " mm (" << metrics.average << " dist) | "
                      << "Calib Lvl: " << calibLevel << " mm (" << calibratedDist << " dist) | "
                      << "Yield: " << metrics.validSamples << "/3       " << std::flush;
        } else {
            std::cout << "\r[SCATTERED]                                                                    " << std::flush;
        }
    }

    if (kbhit()) getchar();
    std::cout << std::defaultfloat;
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
        sensor.setMeasurementTimingBudget(200000); 
    } catch (const std::exception & error) {
        std::cerr << "Error initializing ToF sensor: " << error.what() << std::endl;
        return 2;
    }

    uint16_t containerZero = loadContainerZero();
    int choice = 0;

    std::cout << "\n=========================================" << std::endl;
    std::cout << "  MIXR-1: MASTER WATER TESTING SUITE     " << std::endl;
    std::cout << "  Session ID: " << currentSessionID << std::endl;
    std::cout << "=========================================" << std::endl;
    
    if (containerZero != 0) {
        float calibDisp = (static_cast<float>(containerZero) - 9.067f) / 1.052f;
        std::cout << "Loaded saved container zero: " << containerZero << " mm raw (" 
                  << std::fixed << std::setprecision(2) << calibDisp << " mm calib)." << std::defaultfloat << std::endl;
    }

    while (!systemOffline) {
        std::cout << "\nSelect an operation:" << std::endl;
        std::cout << "  [1] Set Container Bottom (Zero Calibration)" << std::endl;
        std::cout << "  [2] Start Pump Fill Sequence (Live Control)" << std::endl;
        std::cout << "  [3] Record Static Data Point (Log to CSV)" << std::endl;
        std::cout << "  [4] Continuous Sensor Stream" << std::endl;
        std::cout << "  [5] Exit System" << std::endl;
        std::cout << "Selection: ";
        
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
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
                runRecordDataPoint(sensor, containerZero);
                break;
            case 4:
                runContinuousRead(sensor, containerZero);
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
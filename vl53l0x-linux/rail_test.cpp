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

// BCM Pin Definitions
#define MOTOR_INA 17
#define MOTOR_INB 27
#define MOTOR_PWM 13

volatile sig_atomic_t systemOffline = 0;

const char* DATA_FILE = "tof_rail_calibration.csv";

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

// Hardware Interrupt Handler
void sigintHandler(int) {
    systemOffline = 1;
}

// Failsafe Pump Shutdown
void stopPump() {
    digitalWrite(MOTOR_INA, LOW);
    digitalWrite(MOTOR_INB, LOW);
    pwmWrite(MOTOR_PWM, 0);
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

// Checks if the CSV file already exists
bool fileExists(const char* filename) {
    std::ifstream f(filename);
    return f.good();
}

// Collects a burst of readings and extracts precision/reliability metrics
SensorMetrics getSensorMetrics(VL53L0X& sensor, int samples) {
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
            // Ignore single I2C dropouts during filtering burst
        }
        usleep(10000); // 10ms delay between filter bursts
    }

    metrics.validSamples = validReadings.size();

    if (metrics.validSamples == 0) {
        metrics.min = 0; // Reset min if no readings were taken
        return metrics;
    }

    long sum = std::accumulate(validReadings.begin(), validReadings.end(), 0);
    metrics.average = static_cast<uint16_t>(sum / metrics.validSamples);
    
    return metrics;
}

// Appends full metrics to CSV
void logToCSV(const std::string& sessionID, int testNumber, int trialNumber, int targetDistance, const SensorMetrics& metrics) {
    bool writeHeader = !fileExists(DATA_FILE);
    std::ofstream file(DATA_FILE, std::ios::app);
    
    int absoluteError = static_cast<int>(metrics.average) - targetDistance;
    
    if (file.is_open()) {
        if (writeHeader) {
            file << "Session_ID,Test_Number,Trial_Number,Target_Distance_mm,Sensor_Avg_mm,Absolute_Error_mm,Min_Read_mm,Max_Read_mm,Valid_Samples,Target_Samples\n";
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
             << metrics.targetSamples << "\n";
             
        std::cout << "[SYSTEM] Logged -> Avg: " << metrics.average << " mm | Error: " << (absoluteError > 0 ? "+" : "") << absoluteError 
                  << " mm | Spread: [" << metrics.min << " to " << metrics.max << "] | Yield: " 
                  << metrics.validSamples << "/" << metrics.targetSamples << "\n";
    } else {
        std::cerr << "[!] CRITICAL: Could not open " << DATA_FILE << " for writing.\n";
    }
}

// Mode 1: Multi-Trial Record and Log
void runRecordDataPoint(VL53L0X& sensor) {
    std::cout << "\n--- [ DATA RECORDING ] ---" << std::endl;
    
    int targetDistance;
    std::cout << "Enter the physical target distance on rail (mm): ";
    if (!(std::cin >> targetDistance)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "[!] Invalid input. Returning to menu." << std::endl;
        return;
    }

    int trials;
    std::cout << "Enter number of trials for this distance (e.g., 5): ";
    if (!(std::cin >> trials)) {
        std::cin.clear();
        std::cin.ignore(10000, '\n');
        std::cout << "[!] Invalid input. Returning to menu." << std::endl;
        return;
    }
    
    // Clear the input buffer before utilizing getline for the ENTER key pauses
    std::cin.ignore(10000, '\n');

    for (int t = 1; t <= trials; ++t) {
        if (t > 1) {
            std::cout << "\n[Trial " << t << "/" << trials << "] Slide board OFF the mark, then slide it BACK IN to " << targetDistance << " mm." << std::endl;
        } else {
            std::cout << "\n[Trial " << t << "/" << trials << "] Ensure board is secured at " << targetDistance << " mm." << std::endl;
        }
        
        std::cout << "Press ENTER to capture 10-sample burst...";
        std::string dummy;
        std::getline(std::cin, dummy);

        SensorMetrics metrics = getSensorMetrics(sensor, 10);

        if (metrics.validSamples == 0) {
            std::cout << "[!] ERROR: Failed to get any valid readings. Try this trial again." << std::endl;
            t--; // Decrement loop counter to retry this specific trial
            continue;
        }

        logToCSV(currentSessionID, currentTestNumber, t, targetDistance, metrics);
    }
    
    currentTestNumber++; // Corrected: Increments only after the target distance batch is finished
    std::cout << "\n>> Finished all " << trials << " trials for " << targetDistance << " mm." << std::endl;
}

// Mode 2: Continuous Read for Alignment
void runContinuousRead(VL53L0X& sensor) {
    std::cout << "\n--- [ CONTINUOUS READ ] ---" << std::endl;
    std::cout << "Streaming sensor data. Press ANY KEY to stop and return to menu.\n\n";

    // Flush any pending stdin characters
    while (kbhit()) getchar();

    while (!systemOffline && !kbhit()) {
        try {
            uint16_t dist = sensor.readRangeSingleMillimeters();
            if (!sensor.timeoutOccurred() && dist > 0 && dist < 2000) {
                std::cout << "\rCurrent Sensor Reading: " << dist << " mm      " << std::flush;
            } else {
                std::cout << "\rCurrent Sensor Reading: [OUT OF RANGE]      " << std::flush;
            }
        } catch (...) {
            std::cout << "\rCurrent Sensor Reading: [I2C ERROR]         " << std::flush;
        }
        usleep(50000); // 50ms refresh rate for smooth console output
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

    // Generate Session ID at startup
    currentSessionID = generateSessionID();

    VL53L0X sensor;
    try {
        sensor.initialize();
        sensor.setTimeout(500);
        sensor.setMeasurementTimingBudget(200000); // High accuracy mode
    } catch (const std::exception & error) {
        std::cerr << "Error initializing ToF sensor: " << error.what() << std::endl;
        return 2;
    }

    int choice = 0;

    std::cout << "\n=========================================" << std::endl;
    std::cout << " MIXR-1: TOF EXPERIMENTAL RANGE FINDER  " << std::endl;
    std::cout << " Session ID: " << currentSessionID << std::endl;
    std::cout << "=========================================" << std::endl;

    while (!systemOffline) {
        std::cout << "\nSelect an operation:" << std::endl;
        std::cout << "  [1] Read (Take 10 samples and log metrics)" << std::endl;
        std::cout << "  [2] Continuous Read (Target alignment)" << std::endl;
        std::cout << "  [3] Exit System" << std::endl;
        std::cout << "Selection: ";
        
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            break; 
        }

        switch (choice) {
            case 1:
                runRecordDataPoint(sensor);
                break;
            case 2:
                runContinuousRead(sensor);
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
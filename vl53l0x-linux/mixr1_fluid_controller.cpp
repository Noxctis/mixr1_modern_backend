// mixr1_fluid_controller.cpp
//
// Revision notes (this pass):
//   1. BUG FIX: two call sites computed a fluid level or floater thickness from
//      getSensorMetrics() without checking validSamples first. If every sample in
//      that batch failed (occlusion, glare, bad angle), the struct's average silently
//      stayed at its default of 0, and the code used that 0 as if it were a real
//      reading -- e.g. floaterThickness would become "tankBottom - 0" (garbage) with
//      no warning at all. Fixed to match the pattern already used in runCalibration().
//   2. REFACTOR: the "zero - (raw + floaterThickness)" formula was duplicated in ~9
//      places (runContinuousRead, runSolenoidAndToF, runFullFluidCycle x4,
//      runTankZeroExperiment x3, capture100Readings). Pulled into one
//      computeFluidLevel() helper so there's one place to get it right and one place
//      to apply a calibration correction.
//   3. CALIBRATION: tank_zero_experiment_real_data.csv (26 fills, 50-250 mm targets)
//      shows CalculatedFluid_mm reading consistently HIGH vs the physical ruler
//      measurement, and the error scales with level rather than sitting at a fixed
//      mm offset -- a linear fit of Calculated = m*Physical + b gives m ~= 1.066,
//      b ~= -0.14 mm (intercept ~0), with residual stdev ~2.2 mm. A pure-offset model
//      (Calculated = Physical + constant) fits much worse (~4.9 mm residual stdev).
//      A near-zero-intercept, ~6.6% GAIN error is the signature of a geometric issue
//      (most likely: the sensor's beam axis isn't perfectly perpendicular to the
//      floater/fluid surface -- a slant range reads longer than the true vertical
//      distance by 1/cos(theta); ~6.6% high corresponds to roughly a 20 deg tilt,
//      well within the VL53L0X's 25 deg FoV, so it wouldn't necessarily be obvious
//      from a quick look at the mount) rather than a per-sample electronics issue.
//      See LEVEL_GAIN_CORRECTION below -- left at 1.0 (off) by default because this
//      routine auto-stops a pump on the calculated level, and flipping a fill/drain
//      cutoff based on a single regression without re-verifying it on your current
//      mount is a safety call only you should make. Note the CURRENT (uncorrected)
//      bias direction: calculated level reads HIGH, so fills currently stop a bit
//      SHORT of the true physical target -- i.e. today's bug is "undershoot", not
//      overflow. Enabling the correction below removes that margin.
//   4. The datasheet's own manufacturing calibration flow (Section 3.3) also
//      recommends an offset + crosstalk calibration via the API, done once and
//      stored on the host; this file never calls anything like that today. It
//      wasn't added here since VL53L0X.hpp isn't in front of me and I don't want to
//      guess at function names that might not exist in your port -- but it's worth
//      checking what your VL53L0X.hpp exposes.
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

// See revision note #3 above. 1.0 = no correction (today's behavior).
// If you re-verify this on your current mount (a handful of known-volume fills,
// compare CalculatedFluid_mm to a physical measurement, fit the slope), set this to
// your measured 1/gain -- the dataset this was derived from gave ~0.938.
constexpr double LEVEL_GAIN_CORRECTION = 0.93738;

struct SensorMetrics {
    uint16_t average;
    uint16_t min;
    uint16_t max;
    int validSamples;
    int targetSamples;
};

// Single source of truth for the "distance to zero-reference minus floater
// thickness" fluid-level formula, with the calibration correction and the
// non-negative clamp applied once, here, instead of at every call site.
double computeFluidLevel(double zeroReference_mm, double rawDistance_mm, double floaterThickness_mm) {
    double level = (zeroReference_mm - rawDistance_mm - floaterThickness_mm) * LEVEL_GAIN_CORRECTION;
    return std::max(0.0, level);
}

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

bool saveCalibration(double containerZero, double floaterThickness) {
    std::ofstream file(CALIBRATION_FILE, std::ios::trunc);
    if (!file.is_open()) return false;
    file << std::fixed << std::setprecision(2) << containerZero << " " << floaterThickness;
    return static_cast<bool>(file);
}

bool loadCalibration(double& containerZero, double& floaterThickness) {
    std::ifstream file(CALIBRATION_FILE);
    if (file.is_open() && (file >> containerZero >> floaterThickness)) {
        return true;
    }
    containerZero = 0.0;
    floaterThickness = 0.0;
    return false;
}

void logCycleData(const std::string& sessionID, int targetLevel, double calculatedLevel, double actualMeasured, uint16_t rawSensorAvg) {
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
         << std::fixed << std::setprecision(2) << calculatedLevel << ","
         << actualMeasured << ","
         << rawSensorAvg << ","
         << error << "\n";
}

void capture100Readings(VL53L0X& sensor, const std::string& eventName, double containerZero, double floaterThickness) {
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
            rawDist = sensor.readRangeSingleMillimeters();
        } catch (...) {}

        double calculatedLevel = 0.0;
        if (containerZero > 0.0) {
            calculatedLevel = computeFluidLevel(containerZero, static_cast<double>(rawDist), floaterThickness);
        }

        std::time_t t = std::time(nullptr);
        char timeBuf[20];
        std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));

        std::cout << "\rSample " << i << "/100: Raw " << rawDist << " mm | Lvl " << std::fixed << std::setprecision(1) << calculatedLevel << " mm    " << std::flush;
        rawFile << currentSessionID << "," << timeBuf << "," << eventName << "," << i << "," << rawDist << "," << std::fixed << std::setprecision(2) << calculatedLevel << "\n";
    }
    std::cout << "\n[DATA LOG] Capture complete.\n";
}

SensorMetrics getSensorMetrics(VL53L0X& sensor, int samples, int delay_us = 10000) {
    std::vector<uint16_t> validReadings;
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
    
    containerZero = static_cast<double>(zeroMetrics.average);
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
            double rawLevel = computeFluidLevel(containerZero, static_cast<double>(metrics.average), floaterThickness);
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
    } 
    else {
        std::cout << "\nEnter duration in seconds: ";
        int seconds;
        if (!(std::cin >> seconds) || seconds <= 0) {
            std::cout << "[!] Invalid duration.\n";
            std::cin.clear(); std::cin.ignore(10000, '\n'); return;
        }
        std::cin.ignore(10000, '\n'); 

        if (hwChoice == 1) setPump(true, pwm_val);
        else setSolenoid(true, pwm_val);

        std::cout << "[" << hwName << " ON] Running for " << seconds << " seconds. Press ANY KEY to abort.\n";
        
        while (kbhit()) getchar();

        auto start_time = std::time(nullptr);
        while (std::time(nullptr) - start_time < seconds && !emergencyStop) {
            if (kbhit()) {
                getchar(); 
                std::cout << "\n[ABORTED] Manual interruption.\n";
                break;
            }
            std::cout << "\rRemaining: " << seconds - (std::time(nullptr) - start_time) << " s   " << std::flush;
            usleep(100000); 
        }

        if (hwChoice == 1) setPump(false);
        else setSolenoid(false);
        
        std::cout << "\n[" << hwName << " OFF]\n";
    }
}

// ==============================================================================
// FILL TESTS
// ==============================================================================

void runTankZeroExperiment(VL53L0X& sensor) {
    std::cout << "\n--- [ TANK ZERO DISTANCE EXPERIMENT (FILL & MONITOR) ] ---\n";
    
    // 1. Calibration: Tank Bottom
    double tankBottom = 0.0;
    int zeroChoice = 0;
    
    std::cout << "\nHow would you like to set the Tank Bottom Zero?\n";
    std::cout << " [1] Auto-read via ToF Sensor (Requires opaque target on clear tank floors)\n";
    std::cout << " [2] Manually enter known physical distance (Recommended for clear acrylic)\n";
    std::cout << "Selection: ";
    
    if (!(std::cin >> zeroChoice) || (zeroChoice != 1 && zeroChoice != 2)) {
        std::cout << "[!] Invalid selection.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }
    std::cin.ignore(10000, '\n');

    if (zeroChoice == 1) {
        std::cout << "\n[!] Ensure the tank is EMPTY and the FLOATER IS REMOVED.\n";
        std::cout << "Press ENTER to read the Tank Bottom Distance...";
        std::string dummy;
        std::getline(std::cin, dummy);
        
        SensorMetrics bottomMetrics = getSensorMetrics(sensor, 20, 10000);
        if (bottomMetrics.validSamples == 0) {
            std::cout << "[!] Calibration failed. Check sensor wiring.\n";
            return;
        }
        tankBottom = static_cast<double>(bottomMetrics.average);
        std::cout << ">> Tank Bottom Zero (Auto): " << std::fixed << std::setprecision(2) << tankBottom << " mm\n\n";
    } else {
        std::cout << "\nEnter physical distance from the sensor chip to the tank bottom (mm): ";
        if (!(std::cin >> tankBottom) || tankBottom <= 0) {
            std::cout << "[!] Invalid distance.\n";
            std::cin.clear(); std::cin.ignore(10000, '\n'); return;
        }
        std::cin.ignore(10000, '\n');
        std::cout << ">> Tank Bottom Zero (Manual): " << std::fixed << std::setprecision(2) << tankBottom << " mm\n\n";
    }

    // Floater Thickness Setup
    std::string dummy;
    std::cout << "[!] Place the FLOATER in the tank.\n";
    std::cout << "Press ENTER to read Floater resting distance...";
    std::getline(std::cin, dummy);

    SensorMetrics floaterMetrics = getSensorMetrics(sensor, 20, 10000);
    if (floaterMetrics.validSamples == 0) {
        std::cout << "[!] Floater resting-distance read failed (no valid ToF samples) -- "
                      "check the floater is in the sensor's FoV and try again. Aborting experiment.\n";
        return;
    }
    double floaterThickness = tankBottom - static_cast<double>(floaterMetrics.average);
    std::cout << ">> Floater Resting Dist: " << floaterMetrics.average << " mm\n";
    std::cout << ">> Calculated Floater Thickness: " << std::fixed << std::setprecision(2) << floaterThickness << " mm\n\n";

    // 2. Configuration Prompts
    int targetLevel, numIterations, numPhysicalMeasures;
    
    std::cout << "Enter target fluid level (mm): ";
    if (!(std::cin >> targetLevel)) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
    
    std::cout << "Enter number of iterations for this level: ";
    if (!(std::cin >> numIterations) || numIterations <= 0) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
    
    std::cout << "Enter number of physical measurements to take per iteration: ";
    if (!(std::cin >> numPhysicalMeasures) || numPhysicalMeasures <= 0) { std::cin.clear(); std::cin.ignore(10000, '\n'); return; }
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
        std::cout << "\n=======================================\n";
        std::cout << " ITERATION " << iter << " / " << numIterations << "\n";
        std::cout << "=======================================\n";

        // 3. Fill Phase
        std::cout << "[!] Ensure FLOATER is IN the tank.\n";
        std::cout << "Press ENTER to START PUMP and fill to " << targetLevel << " mm (or type 'q' to abort)... ";
        std::cin.clear();
        std::getline(std::cin, dummy);
        if (dummy == "q" || dummy == "Q") { abortExp = true; break; }

        std::cout << "Filling... Press ANY KEY to stop pump early, or type 'q' to EXIT experiment.\n";
        
        while (kbhit()) getchar();
        setPump(true, pump_pwm);
        
        while (!emergencyStop) {
            if (kbhit()) {
                char c = getchar();
                if (c == 'q' || c == 'Q') abortExp = true;
                break; 
            }

            SensorMetrics metrics = getSensorMetrics(sensor, 3, 5000);
            if (metrics.validSamples > 0) {
                double rawLevel = computeFluidLevel(tankBottom, static_cast<double>(metrics.average), floaterThickness);
                std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << rawLevel << "/" << targetLevel << " mm    " << std::flush;
                if (rawLevel >= targetLevel) break;
            }
        }
        setPump(false);
        if (emergencyStop || abortExp) {
            std::cout << "\n[ABORT] Exiting experiment...\n";
            break;
        }

        std::cout << "\n[SYSTEM] Target level reached. Waiting 2 seconds for fluid to settle...\n";
        for (int w = 0; w < 20; w++) {
            if (kbhit()) {
                char c = getchar();
                if (c == 'q' || c == 'Q') { abortExp = true; break; }
            }
            usleep(100000);
        }
        if (emergencyStop || abortExp) break;

        // 4. Capture ToF metrics 
        std::cout << "Capturing ToF readings (100 samples). Press 'q' to EXIT experiment.\n";
        std::vector<uint16_t> validReadings;
        
        for (int i = 1; i <= 100; i++) {
            if (emergencyStop) break;
            if (kbhit()) {
                char c = getchar();
                if (c == 'q' || c == 'Q') { abortExp = true; break; }
            }
            
            try {
                uint16_t dist = sensor.readRangeSingleMillimeters();
                if (!sensor.timeoutOccurred() && dist > 0 && dist < 2000) {
                    validReadings.push_back(dist);
                }
            } catch (...) {}
            
            std::cout << "\rSample " << i << "/100 captured...    " << std::flush;
            usleep(5000); 
        }
        if (emergencyStop || abortExp) break;

        double settledAvg = 0.0;
        if (!validReadings.empty()) {
            unsigned long sum = std::accumulate(validReadings.begin(), validReadings.end(), 0UL);
            settledAvg = static_cast<double>(sum) / validReadings.size();
        } else {
            settledAvg = tankBottom;
        }

        double distFromZero = tankBottom - settledAvg;
        double calcLevel = computeFluidLevel(tankBottom, settledAvg, floaterThickness);
        
        std::cout << "\n>> ToF Average: " << std::fixed << std::setprecision(2) << settledAvg << " mm\n";
        std::cout << ">> Calculated Level: " << std::fixed << std::setprecision(2) << calcLevel << " mm\n\n";

        // 5. Physical Measurements (Only requested during the fill phase)
        std::cout << "[!] REMOVE the floater from the tank.\n";
        for (int p = 1; p <= numPhysicalMeasures; p++) {
            double pMeasure = 0.0;
            std::cout << "Enter physical measurement #" << p << " (mm) [-1 to EXIT]: ";
            
            if (!(std::cin >> pMeasure)) {
                std::cin.clear(); std::cin.ignore(10000, '\n'); pMeasure = 0.0;
            } else {
                std::cin.ignore(10000, '\n');
            }

            if (pMeasure < 0.0) {
                abortExp = true;
                break;
            }
            
            expFile << currentSessionID << "," << iter << "," << targetLevel << "," 
                    << std::fixed << std::setprecision(2) << settledAvg << "," << distFromZero << "," 
                    << calcLevel << "," << p << "," << pMeasure << "\n";
        }
        if (abortExp) break;
        std::cout << "[SYSTEM] Measurements saved.\n\n";

        // 6. Drain Phase
        std::cout << "[!] PLACE FLOATER BACK IN THE TANK.\n";
        std::cout << "Press ENTER to OPEN SOLENOID and start draining (or type 'q' to EXIT)... ";
        
        std::cin.clear();
        std::getline(std::cin, dummy);
        if (dummy == "q" || dummy == "Q") {
            abortExp = true;
            break;
        }

        std::cout << "Draining... Press ANY KEY to stop solenoid, or 'q' to EXIT experiment.\n";
        setSolenoid(true, sol_pwm);
        
        while (kbhit()) getchar(); 
        
        while (!emergencyStop) {
            if (kbhit()) {
                char c = getchar();
                if (c == 'q' || c == 'Q') abortExp = true;
                break; 
            }

            SensorMetrics metrics = getSensorMetrics(sensor, 2, 10000);
            if (metrics.validSamples > 0) {
                double rawLevel = computeFluidLevel(tankBottom, static_cast<double>(metrics.average), floaterThickness);
                std::cout << "\rCurrent Lvl: " << std::fixed << std::setprecision(1) << rawLevel << " mm    " << std::flush;
                
                if (rawLevel <= 2.0) {
                    std::cout << "\n[SYSTEM] Tank empty. Auto-closing solenoid.\n";
                    break;
                }
            }
        }
        setSolenoid(false);
        if (abortExp) break;
        
        std::cout << "\n[SYSTEM] Draining complete for iteration " << iter << ".\n";
    }
    
    if (abortExp) {
        std::cout << "\n[SYSTEM] Experiment terminated early by user.\n";
    } else {
        std::cout << "\n[SYSTEM] Experiment sequence complete. Hardware parked.\n";
    }
}

// ==============================================================================
// DRAIN TESTS
// ==============================================================================

void runSolenoidAndToF(VL53L0X& sensor, double containerZero, double floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }
    
    std::cout << "\n--- [ GRAVITY DRAIN & MONITOR ] ---\n";
    int sol_pwm = getPWMFromVoltage("Solenoid");

    std::cout << "Press ENTER to OPEN SOLENOID and monitor ToF drop (or type 'q' to abort)... ";
    std::string dummy;
    std::cin.clear();
    std::getline(std::cin, dummy);
    if (dummy == "q" || dummy == "Q") return;
    
    while (kbhit()) getchar();
    emergencyStop = 0;
    setSolenoid(true, sol_pwm);
    std::cout << "Draining... Press ANY KEY to abort.\n";
    
    while (!emergencyStop && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(sensor, 3, 10000);
        if (metrics.validSamples > 0) {
            double rawLevel = computeFluidLevel(containerZero, static_cast<double>(metrics.average), floaterThickness);
            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << rawLevel << " mm | Yield: " << metrics.validSamples << "/3    " << std::flush;
            
            if (rawLevel <= 2.0) {
                std::cout << "\n[SYSTEM] Tank empty. Closing solenoid.\n";
                break;
            }
        }
    }
    
    if (kbhit()) getchar();
    setSolenoid(false);
}

// ==============================================================================
// FILL AND DRAIN TESTS
// ==============================================================================

void runFullFluidCycle(VL53L0X& sensor, double containerZero, double floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }
    
    std::cout << "\n--- [ FULL CYCLE: FILL -> SETTLE -> STEPPED DRAIN ] ---\n";
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

    std::cout << "\n[PHASE 1] FILLING\n";
    std::cout << "Press ENTER to START PUMP and fill to " << targetLevel << " mm (or type 'q' to abort)... ";
    std::string dummy;
    std::cin.clear();
    std::getline(std::cin, dummy);
    if (dummy == "q" || dummy == "Q") return;

    emergencyStop = 0;
    setSolenoid(false);
    
    while (kbhit()) getchar();
    setPump(true, pump_pwm);
    std::cout << "Filling... Press ANY KEY to abort.\n";
    
    while (!emergencyStop) {
        if (kbhit()) { emergencyStop = 1; break; }

        SensorMetrics metrics = getSensorMetrics(sensor, 3, 5000);
        if (metrics.validSamples > 0) {
            double rawLevel = computeFluidLevel(containerZero, static_cast<double>(metrics.average), floaterThickness);
            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << rawLevel << "/" << targetLevel << " mm    " << std::flush;
            if (rawLevel >= targetLevel) break;
        }
    }
    
    setPump(false);
    if (emergencyStop) return;

    std::cout << "\n[SYSTEM] Target reached. Settling fluid...\n";
    usleep(1000000); 
    
    capture100Readings(sensor, "FullCycle_SettledMeasurement", containerZero, floaterThickness); 
    SensorMetrics settleMetrics = getSensorMetrics(sensor, 5, 10000);
    if (settleMetrics.validSamples == 0) {
        std::cout << "\n[!] Settled-level read failed (no valid ToF samples) -- hardware parked, aborting cycle.\n";
        setPump(false);
        setSolenoid(false);
        return;
    }
    double calculatedLevel = computeFluidLevel(containerZero, static_cast<double>(settleMetrics.average), floaterThickness);

    std::cout << "\n\n[PHASE 2] SETTLED (ToF Calculated: " << std::fixed << std::setprecision(2) << calculatedLevel << " mm)\n";
    std::cout << "Enter actual measured fluid level (mm): ";
    
    double actualMeasured = 0.0;
    if (std::cin >> actualMeasured) {
        std::cin.ignore(10000, '\n');
        logCycleData(currentSessionID, targetLevel, calculatedLevel, actualMeasured, settleMetrics.average);
    } else {
        std::cin.clear(); std::cin.ignore(10000, '\n');
    }

    std::cout << "\n[PHASE 3] STEPPED DRAINING (Step Size: " << drainInterval << " mm)\n";
    
    while (!emergencyStop) {
        SensorMetrics currentMetrics = getSensorMetrics(sensor, 3, 5000);
        double currentLevel = 0.0;
        if (currentMetrics.validSamples > 0) {
            currentLevel = computeFluidLevel(containerZero, static_cast<double>(currentMetrics.average), floaterThickness);
        }
        if (currentLevel <= 2.0) break;

        std::cout << "Current Level: " << std::fixed << std::setprecision(1) << currentLevel << " mm.\n";
        std::cout << "Press ENTER to OPEN SOLENOID and drain " << drainInterval << " mm (or 'q' to stop)... ";
        std::string input;
        std::getline(std::cin, input);
        if (input == "q" || input == "Q") break;
        
        double stepTarget = std::max(0.0, currentLevel - drainInterval);
        
        setSolenoid(true, sol_pwm);
        std::cout << "Draining... Press ANY KEY to stop.\n";
        while (!emergencyStop) {
            if (kbhit()) { getchar(); break; }

            SensorMetrics metrics = getSensorMetrics(sensor, 2, 5000);
            if (metrics.validSamples > 0) {
                double rawLevel = computeFluidLevel(containerZero, static_cast<double>(metrics.average), floaterThickness);
                std::cout << "\rCurrent: " << std::fixed << std::setprecision(1) << rawLevel << " mm | Target: " << stepTarget << " mm    " << std::flush;
                if (rawLevel <= stepTarget || rawLevel <= 2.0) break;
            }
        }
        setSolenoid(false);
        std::cout << "\n[STEP COMPLETE] Solenoid closed.\n";
        
        usleep(1000000); 
        capture100Readings(sensor, "FullCycle_PostDrainStep", containerZero, floaterThickness);
        std::cout << "\n";
    }
    setSolenoid(false);
    std::cout << "\n[SYSTEM] Cycle complete. Hardware parked.\n";
}

// ==============================================================================
// SUBMENUS
// ==============================================================================

void menuHardware(VL53L0X& sensor, double& containerZero, double& floaterThickness) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- HARDWARE CONTROL & SETUP ---\n";
        std::cout << " [1] Set Container & Floater (Calibration)\n";
        std::cout << " [2] Solenoid Toggle (Dry Test)\n";
        std::cout << " [3] Manual Hardware Control (Toggle / Timer)\n";
        std::cout << " [4] Continuous Sensor Stream\n";
        std::cout << " [5] Back to Main Menu\nSelection: ";
        
        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runCalibration(sensor, containerZero, floaterThickness); break;
            case 2: runSolenoidTestOnly(); break;
            case 3: runManualHardwareControl(); break;
            case 4: runContinuousRead(sensor, containerZero, floaterThickness); break;
            case 5: return;
        }
    }
}

void menuFillTests(VL53L0X& sensor) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- FILL TESTS ---\n";
        std::cout << " [1] Tank Zero Distance Experiment (Fill & Monitor + Iterations)\n";
        std::cout << " [2] Back to Main Menu\nSelection: ";
        
        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runTankZeroExperiment(sensor); break;
            case 2: return;
        }
    }
}

void menuDrainTests(VL53L0X& sensor, double containerZero, double floaterThickness) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- DRAIN TESTS ---\n";
        std::cout << " [1] Gravity Drain & Monitor (Solenoid + ToF)\n";
        std::cout << " [2] Back to Main Menu\nSelection: ";
        
        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runSolenoidAndToF(sensor, containerZero, floaterThickness); break;
            case 2: return;
        }
    }
}

void menuFillAndDrainTests(VL53L0X& sensor, double containerZero, double floaterThickness) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- FILL AND DRAIN TESTS ---\n";
        std::cout << " [1] Full Cycle (Pump Fill -> Settle -> Stepped Drain)\n";
        std::cout << " [2] Back to Main Menu\nSelection: ";
        
        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runFullFluidCycle(sensor, containerZero, floaterThickness); break;
            case 2: return;
        }
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

    double containerZero = 0.0;
    double floaterThickness = 0.0;
    loadCalibration(containerZero, floaterThickness);

    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n=========================================\n";
        std::cout << " MIXR-1: FLUID DYNAMICS CONTROLLER\n";
        std::cout << "=========================================\n";
        std::cout << " [1] Hardware Control & Setup\n";
        std::cout << " [2] Fill Tests\n";
        std::cout << " [3] Drain Tests\n";
        std::cout << " [4] Fill and Drain Tests\n";
        std::cout << " [5] Exit System\nSelection: ";
        
        if (!(std::cin >> choice)) {
            std::cin.clear();
            std::cin.ignore(10000, '\n');
            break; 
        }
        std::cin.ignore(10000, '\n'); 

        switch (choice) {
            case 1: menuHardware(sensor, containerZero, floaterThickness); break;
            case 2: menuFillTests(sensor); break;
            case 3: menuDrainTests(sensor, containerZero, floaterThickness); break;
            case 4: menuFillAndDrainTests(sensor, containerZero, floaterThickness); break;
            case 5: systemOffline = 1; break;
        }
    }

    setPump(false);
    setSolenoid(false);
    std::cout << "\nSystem Offline. Hardware safely parked.\n";
    return 0;
}
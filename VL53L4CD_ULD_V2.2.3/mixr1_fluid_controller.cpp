// mixr1_fluid_controller.cpp  (drain-accuracy revision)
//
// Changes this pass:
//   1. Sensor readings keep sub-mm precision (double) and use a median-based
//      robust average instead of an integer-truncated mean.
//   2. Stale-reading flush before "settled" measurements.
//   3. New drainToLevel(): fast 50 ms timing while draining, rate estimate,
//      predictive valve close (compensates measurement latency), stall watchdog.
//   4. Drain-to-empty now detects the floater landing on the gasket and keeps
//      the valve open for a final dwell, because the ToF cannot see fluid that
//      is below the floater's draft.
//   5. trimToLevel(): after a stepped drain, re-measure at full accuracy and
//      pulse the valve if still above target.
//   6. Removed SetOffset call: a constant sensor offset cancels in
//      (zero - raw), so it never affected the level.

#include <wiringPi.h>
#include <iostream>
#include <fstream>
#include <csignal>
#include <unistd.h>
#include <vector>
#include <deque>
#include <numeric>
#include <termios.h>
#include <fcntl.h>
#include <ctime>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <algorithm>

#include "VL53L4CD_ULD_Driver/VL53L4CD_api.h"
#include "Platform/platform.h"

uint8_t VL53L4CD_PlatformInit(void);

// BCM Pin Definitions - PUMP (VNH5019 #1)
constexpr int PUMP_INA = 17;
constexpr int PUMP_INB = 27;
constexpr int PUMP_PWM = 13;
constexpr int PUMP_EN  = 15;

// BCM Pin Definitions - SOLENOID (VNH5019 #2)
constexpr int SOLENOID_INA = 5;
constexpr int SOLENOID_INB = 6;
constexpr int SOLENOID_PWM = 12;

volatile sig_atomic_t systemOffline = 0;
volatile sig_atomic_t emergencyStop = 0;

const char* CALIBRATION_FILE = "container_zero.txt";
const char* DATA_FILE = "fluid_dynamics_data.csv";
const char* RAW_DATA_FILE = "raw_sensor_data.csv";

std::string currentSessionID;

// ---- Level calibration (fit these from your CSV: actual = GAIN*calc + OFFSET) ----
constexpr double LEVEL_GAIN_CORRECTION      = 1.0;
constexpr double LEVEL_OFFSET_CORRECTION_MM = 0.0;

// ---- Sensor timing ----
constexpr uint32_t TIMING_ACCURATE_MS = 200;  // settled measurements
constexpr uint32_t TIMING_FAST_MS     = 50;   // closed-loop drain control
constexpr double   OUTLIER_BAND_MM    = 3.0;  // samples farther than this from median are dropped

// ---- Drain tuning ----
constexpr double EMPTY_BAND_MM        = 1.5;  // |level| <= this => at the floor
constexpr int    EMPTY_CONFIRM_READS  = 5;    // consecutive reads in band before "empty"
constexpr double FLOOR_MAX_MM         = 10.0; // floor-stall detection only below this level
constexpr double FLOOR_STALL_S        = 3.0;  // no progress this long (below FLOOR_MAX_MM) => floater landed
constexpr double FINAL_DRAIN_DWELL_S  = 6.0;  // keep valve open after floor reached (drains fluid below floater draft)
constexpr double DRAIN_LATENCY_S      = 0.35; // measurement + valve latency used for predictive close
constexpr double RATE_WINDOW_S        = 1.0;  // window for drain-rate estimate
constexpr double DRAIN_STALL_TIMEOUT_S = 20.0;// no progress at higher level => clog/air lock
constexpr double STEP_TOLERANCE_MM    = 1.0;  // accept stepped drain within this of target
constexpr int    MAX_TRIM_PULSES      = 6;
constexpr int    TRIM_MS_PER_MM       = 100;  // pulse length per mm of excess (tune to your valve)
constexpr int    TRIM_MIN_MS          = 80;
constexpr int    TRIM_MAX_MS          = 600;
constexpr int    TRIM_SETTLE_US       = 1500000;

struct SensorMetrics {
    double   average;   // robust mean (outliers around median removed)
    double   median;
    uint16_t min;
    uint16_t max;
    int      validSamples;
    int      targetSamples;
};

enum class DrainResult { Reached, Aborted, Stalled, SensorLost };

// Single source of truth for fluid-level formula
double computeFluidLevel(double zeroReference_mm, double rawDistance_mm, double floaterThickness_mm, bool clampZero = true) {
    double level = (zeroReference_mm - rawDistance_mm - floaterThickness_mm) * LEVEL_GAIN_CORRECTION
                   + LEVEL_OFFSET_CORRECTION_MM;
    return clampZero ? std::max(0.0, level) : level;
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
    file << std::fixed << std::setprecision(3) << containerZero << " " << floaterThickness;
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

void logCycleData(const std::string& sessionID, int targetLevel, double calculatedLevel, double actualMeasured, double rawSensorAvg) {
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

// ------------------------------------------------------------------------------
// ST API WRAPPERS
// ------------------------------------------------------------------------------

// Throw away whatever result is latched and wait for a fresh one.
static void discardStale(Dev_t dev) {
    VL53L4CD_ClearInterrupt(dev);
    uint8_t ready = 0;
    while (!ready && !emergencyStop) {
        VL53L4CD_CheckForDataReady(dev, &ready);
        usleep(1000);
    }
    VL53L4CD_ClearInterrupt(dev);
}

// Change timing budget (must be done with ranging stopped).
void setTiming(Dev_t dev, uint32_t budget_ms) {
    VL53L4CD_StopRanging(dev);
    VL53L4CD_SetRangeTiming(dev, budget_ms, 0);
    VL53L4CD_StartRanging(dev);
}

// RAII: fast timing while in scope, accurate timing restored on exit.
struct FastTimingScope {
    Dev_t dev;
    explicit FastTimingScope(Dev_t d) : dev(d) { setTiming(dev, TIMING_FAST_MS); }
    ~FastTimingScope() { setTiming(dev, TIMING_ACCURATE_MS); }
};

SensorMetrics getSensorMetrics(Dev_t dev, int samples, int delay_us = 10000, bool flushFirst = false) {
    std::vector<uint16_t> validReadings;
    SensorMetrics metrics = {0.0, 0.0, 65535, 0, 0, samples};
    VL53L4CD_ResultsData_t results;
    uint8_t dataReady = 0;

    if (flushFirst) discardStale(dev);

    for (int i = 0; i < samples; ++i) {
        if (emergencyStop) break;

        dataReady = 0;
        while (!dataReady && !emergencyStop) {
            VL53L4CD_CheckForDataReady(dev, &dataReady);
            usleep(1000);
        }

        if (emergencyStop) break;

        VL53L4CD_GetResult(dev, &results);
        VL53L4CD_ClearInterrupt(dev);

        uint16_t dist = results.distance_mm;

        if (results.range_status == 0 && dist > 0 && dist < 2000) {
            validReadings.push_back(dist);
            if (dist < metrics.min) metrics.min = dist;
            if (dist > metrics.max) metrics.max = dist;
        }
        if (delay_us > 0) usleep(delay_us);
    }

    metrics.validSamples = static_cast<int>(validReadings.size());
    if (metrics.validSamples == 0) {
        metrics.min = 0;
        return metrics;
    }

    std::vector<uint16_t> sorted = validReadings;
    std::sort(sorted.begin(), sorted.end());
    size_t n = sorted.size();
    double med = (n % 2) ? sorted[n / 2] : (sorted[n / 2 - 1] + sorted[n / 2]) / 2.0;

    double sum = 0.0;
    int cnt = 0;
    for (uint16_t v : validReadings) {
        if (std::fabs(v - med) <= OUTLIER_BAND_MM) { sum += v; cnt++; }
    }
    metrics.median = med;
    metrics.average = cnt ? sum / cnt : med;
    return metrics;
}

// Quick level read for closed-loop control (median of a few fast samples, unclamped).
bool readLevel(Dev_t dev, double zero, double floater, double& level, int samples = 3) {
    SensorMetrics m = getSensorMetrics(dev, samples, 0);
    if (m.validSamples == 0) return false;
    level = computeFluidLevel(zero, m.median, floater, false);
    return true;
}

void capture100Readings(Dev_t dev, const std::string& eventName, double containerZero, double floaterThickness) {
    std::cout << "\n[DATA LOG] Fluid settling complete. Capturing 100 raw ToF readings (~20 seconds)...\n";
    bool exists = fileExists(RAW_DATA_FILE);
    std::ofstream rawFile(RAW_DATA_FILE, std::ios::app);

    if (!exists) {
        rawFile << "SessionID,Timestamp,Event,SampleIndex,RawDistance_mm,CalculatedLevel_mm\n";
    }

    VL53L4CD_ResultsData_t results;
    uint8_t dataReady = 0;

    discardStale(dev);

    for (int i = 1; i <= 100; i++) {
        if (emergencyStop) break;

        dataReady = 0;
        while (!dataReady && !emergencyStop) {
            VL53L4CD_CheckForDataReady(dev, &dataReady);
            usleep(1000);
        }

        if (emergencyStop) break;

        VL53L4CD_GetResult(dev, &results);
        VL53L4CD_ClearInterrupt(dev);

        uint16_t rawDist = (results.range_status == 0) ? results.distance_mm : 0;

        double calculatedLevel = 0.0;
        if (containerZero > 0.0 && rawDist > 0) {
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

// ------------------------------------------------------------------------------
// DRAIN CONTROL
// ------------------------------------------------------------------------------

// Drain until level <= targetLevel. If targetLevel <= EMPTY_BAND_MM this is a
// drain-to-empty: the loop waits for the floater to land (level stops changing
// near zero), then keeps the valve open for FINAL_DRAIN_DWELL_S so fluid below
// the floater's draft (invisible to the ToF) can still run out.
DrainResult drainToLevel(Dev_t dev, double zero, double floater, int solPwm,
                         double targetLevel, char* abortKey = nullptr) {
    using clock = std::chrono::steady_clock;
    auto secs = [](clock::time_point a, clock::time_point b) {
        return std::chrono::duration<double>(b - a).count();
    };

    const bool toEmpty = targetLevel <= EMPTY_BAND_MM;
    DrainResult result = DrainResult::Reached;
    {
        FastTimingScope fast(dev);
        while (kbhit()) getchar();
        setSolenoid(true, solPwm);

        const auto t0 = clock::now();
        auto tProgress = t0;
        double bestLevel = 1e9;
        std::deque<std::pair<double, double>> hist;   // (t, level)
        int emptyCount = 0, badReads = 0;

        while (!emergencyStop) {
            if (kbhit()) {
                char c = getchar();
                if (abortKey) *abortKey = c;
                result = DrainResult::Aborted;
                break;
            }

            double level;
            if (!readLevel(dev, zero, floater, level)) {
                if (++badReads > 20) { result = DrainResult::SensorLost; break; }
                continue;
            }
            badReads = 0;

            auto now = clock::now();
            double ts = secs(t0, now);

            hist.push_back({ts, level});
            while (hist.size() > 2 && ts - hist.front().first > RATE_WINDOW_S) hist.pop_front();
            double rate = 0.0;   // mm/s, negative while draining
            if (hist.size() >= 3) {
                double dt = ts - hist.front().first;
                if (dt > 0.3) rate = (level - hist.front().second) / dt;
            }

            if (level < bestLevel - 0.5) { bestLevel = level; tProgress = now; }
            double stall = secs(tProgress, now);

            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << level
                      << " mm | Rate: " << rate << " mm/s | Target: " << targetLevel << " mm    " << std::flush;

            if (toEmpty) {
                if (std::fabs(level) <= EMPTY_BAND_MM) {
                    if (++emptyCount >= EMPTY_CONFIRM_READS) break;
                } else {
                    emptyCount = 0;
                }
                if (stall > FLOOR_STALL_S && level <= FLOOR_MAX_MM) break;   // floater landed
            } else {
                double predicted = level + std::min(rate, 0.0) * DRAIN_LATENCY_S;
                if (predicted <= targetLevel) break;
            }

            if (stall > DRAIN_STALL_TIMEOUT_S) { result = DrainResult::Stalled; break; }
        }

        if (toEmpty && result == DrainResult::Reached && !emergencyStop) {
            std::cout << "\n[SYSTEM] Floor reached. Final drain dwell (" << FINAL_DRAIN_DWELL_S << " s)...\n";
            auto tEnd = clock::now() + std::chrono::milliseconds(static_cast<int>(FINAL_DRAIN_DWELL_S * 1000));
            while (!emergencyStop && clock::now() < tEnd) {
                if (kbhit()) {
                    char c = getchar();
                    if (abortKey) *abortKey = c;
                    result = DrainResult::Aborted;
                    break;
                }
                usleep(50000);
            }
        }
        setSolenoid(false);
    }   // accurate timing restored here
    return result;
}

// After a stepped drain: let fluid settle, measure accurately, pulse the valve
// if the level is still above target. Returns true if within tolerance.
bool trimToLevel(Dev_t dev, double zero, double floater, int solPwm, double target) {
    std::cout << "[TRIM] Verifying level at full accuracy...\n";
    for (int i = 0; i <= MAX_TRIM_PULSES && !emergencyStop; ++i) {
        usleep(TRIM_SETTLE_US);
        SensorMetrics m = getSensorMetrics(dev, 8, 0, true);
        if (m.validSamples == 0) return false;

        double level = computeFluidLevel(zero, m.average, floater, false);
        double excess = level - target;
        std::cout << "[TRIM] Level " << std::fixed << std::setprecision(2) << level
                  << " mm (target " << target << ", excess " << excess << ")\n";
        if (excess <= STEP_TOLERANCE_MM) return true;
        if (i == MAX_TRIM_PULSES) break;

        int pulseMs = std::clamp(static_cast<int>(excess * TRIM_MS_PER_MM), TRIM_MIN_MS, TRIM_MAX_MS);
        setSolenoid(true, solPwm);
        usleep(pulseMs * 1000);
        setSolenoid(false);
    }
    std::cout << "[TRIM] Could not reach target within tolerance.\n";
    return false;
}

void reportDrain(DrainResult r) {
    switch (r) {
        case DrainResult::Reached:    std::cout << "\n[SYSTEM] Drain target reached. Solenoid closed.\n"; break;
        case DrainResult::Aborted:    std::cout << "\n[SYSTEM] Drain aborted by user. Solenoid closed.\n"; break;
        case DrainResult::Stalled:    std::cout << "\n[!] No level change for " << DRAIN_STALL_TIMEOUT_S << " s (clog / air lock?). Solenoid closed.\n"; break;
        case DrainResult::SensorLost: std::cout << "\n[!] Lost valid ToF readings. Solenoid closed.\n"; break;
    }
}

// ==============================================================================
// HARDWARE CONTROL FUNCTIONS
// ==============================================================================

void runCalibration(Dev_t dev, double& containerZero, double& floaterThickness) {
    std::string dummy;
    std::cout << "\n--- [ CALIBRATION: SET SYSTEM ZERO ] ---\n";
    std::cout << "[!] Ensure the tank is completely DRAINED.\n";
    std::cout << "[!] Place the FLOATER inside (let it rest naturally on the gasket).\n";
    std::cout << "Press ENTER to set system zero...";

    std::cin.clear();
    std::getline(std::cin, dummy);

    capture100Readings(dev, "Calibration_SystemZero", 0.0, 0.0);

    SensorMetrics zeroMetrics = getSensorMetrics(dev, 50, 0, true);
    if (zeroMetrics.validSamples < 25) {
        std::cout << "[!] Calibration failed (" << zeroMetrics.validSamples << "/50 valid). Check sensor reading.\n";
        return;
    }

    containerZero = zeroMetrics.average;
    floaterThickness = 0.0;

    std::cout << ">> System Zero (Distance to resting floater): " << std::fixed << std::setprecision(2) << containerZero << " mm\n";
    std::cout << ">> Spread: " << zeroMetrics.min << " - " << zeroMetrics.max << " mm ("
              << zeroMetrics.validSamples << "/50 valid)\n";
    if (zeroMetrics.max - zeroMetrics.min > 3) {
        std::cout << "[!] Spread is large. Check floater is steady / surface is flat and re-run.\n";
    }
    saveCalibration(containerZero, floaterThickness);
}

void runContinuousRead(Dev_t dev, double containerZero, double floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }
    std::cout << "\n--- [ CONTINUOUS SENSOR STREAM ] ---\nPress ANY KEY to stop.\n\n";

    while (kbhit()) getchar();

    while (!systemOffline && !kbhit()) {
        SensorMetrics metrics = getSensorMetrics(dev, 1, 0);
        if (metrics.validSamples > 0) {
            double rawLevel = computeFluidLevel(containerZero, metrics.average, floaterThickness);
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

void runTankZeroExperiment(Dev_t dev) {
    std::cout << "\n--- [ TANK ZERO DISTANCE EXPERIMENT (FILL & MONITOR) ] ---\n";

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

        SensorMetrics bottomMetrics = getSensorMetrics(dev, 30, 0, true);
        if (bottomMetrics.validSamples == 0) {
            std::cout << "[!] Calibration failed. Check sensor reading.\n";
            return;
        }
        tankBottom = bottomMetrics.average;
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

    std::string dummy;
    std::cout << "[!] Place the FLOATER in the tank.\n";
    std::cout << "Press ENTER to read Floater resting distance...";
    std::getline(std::cin, dummy);

    SensorMetrics floaterMetrics = getSensorMetrics(dev, 30, 0, true);
    if (floaterMetrics.validSamples == 0) {
        std::cout << "[!] Floater resting-distance read failed (no valid ToF samples). Aborting experiment.\n";
        return;
    }
    double floaterThickness = tankBottom - floaterMetrics.average;
    std::cout << ">> Floater Resting Dist: " << std::fixed << std::setprecision(2) << floaterMetrics.average << " mm\n";
    std::cout << ">> Calculated Floater Thickness: " << std::fixed << std::setprecision(2) << floaterThickness << " mm\n\n";

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

            SensorMetrics metrics = getSensorMetrics(dev, 3, 5000);
            if (metrics.validSamples > 0) {
                double rawLevel = computeFluidLevel(tankBottom, metrics.median, floaterThickness);
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

        capture100Readings(dev, "TankZero_SettledMeasurement", tankBottom, floaterThickness);

        SensorMetrics postSettled = getSensorMetrics(dev, 10, 0, true);
        double settledAvg = (postSettled.validSamples > 0) ? postSettled.average : tankBottom;
        double distFromZero = tankBottom - settledAvg;
        double calcLevel = computeFluidLevel(tankBottom, settledAvg, floaterThickness);

        std::cout << "\n>> ToF Average: " << std::fixed << std::setprecision(2) << settledAvg << " mm\n";
        std::cout << ">> Calculated Level: " << std::fixed << std::setprecision(2) << calcLevel << " mm\n\n";

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

        std::cout << "[!] PLACE FLOATER BACK IN THE TANK.\n";
        std::cout << "Press ENTER to OPEN SOLENOID and start draining (or type 'q' to EXIT)... ";

        std::cin.clear();
        std::getline(std::cin, dummy);
        if (dummy == "q" || dummy == "Q") {
            abortExp = true;
            break;
        }

        std::cout << "Draining... Press ANY KEY to stop solenoid, or 'q' to EXIT experiment.\n";
        char key = 0;
        DrainResult dr = drainToLevel(dev, tankBottom, floaterThickness, sol_pwm, 0.0, &key);
        if (dr == DrainResult::Aborted && (key == 'q' || key == 'Q')) abortExp = true;
        reportDrain(dr);
        if (abortExp || dr == DrainResult::Stalled || dr == DrainResult::SensorLost) break;

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

void runSolenoidAndToF(Dev_t dev, double containerZero, double floaterThickness) {
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

    emergencyStop = 0;
    std::cout << "Draining... Press ANY KEY to abort.\n";
    DrainResult dr = drainToLevel(dev, containerZero, floaterThickness, sol_pwm, 0.0);
    reportDrain(dr);
}

// ==============================================================================
// FILL AND DRAIN TESTS
// ==============================================================================

void runFullFluidCycle(Dev_t dev, double containerZero, double floaterThickness) {
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

        SensorMetrics metrics = getSensorMetrics(dev, 3, 5000);
        if (metrics.validSamples > 0) {
            double rawLevel = computeFluidLevel(containerZero, metrics.median, floaterThickness);
            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << rawLevel << "/" << targetLevel << " mm    " << std::flush;
            if (rawLevel >= targetLevel) break;
        }
    }

    setPump(false);
    if (emergencyStop) return;

    std::cout << "\n[SYSTEM] Target reached. Settling fluid...\n";
    usleep(1000000);

    capture100Readings(dev, "FullCycle_SettledMeasurement", containerZero, floaterThickness);
    SensorMetrics settleMetrics = getSensorMetrics(dev, 10, 0, true);
    if (settleMetrics.validSamples == 0) {
        std::cout << "\n[!] Settled-level read failed (no valid ToF samples) -- hardware parked, aborting cycle.\n";
        setPump(false);
        setSolenoid(false);
        return;
    }
    double calculatedLevel = computeFluidLevel(containerZero, settleMetrics.average, floaterThickness);

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
        SensorMetrics currentMetrics = getSensorMetrics(dev, 5, 0, true);
        double currentLevel = 0.0;
        if (currentMetrics.validSamples > 0) {
            currentLevel = computeFluidLevel(containerZero, currentMetrics.average, floaterThickness);
        }
        if (currentLevel <= EMPTY_BAND_MM) break;

        std::cout << "Current Level: " << std::fixed << std::setprecision(1) << currentLevel << " mm.\n";
        std::cout << "Press ENTER to OPEN SOLENOID and drain " << drainInterval << " mm (or 'q' to stop)... ";
        std::string input;
        std::getline(std::cin, input);
        if (input == "q" || input == "Q") break;

        double stepTarget = std::max(0.0, currentLevel - drainInterval);

        std::cout << "Draining... Press ANY KEY to stop.\n";
        DrainResult dr = drainToLevel(dev, containerZero, floaterThickness, sol_pwm, stepTarget);
        reportDrain(dr);
        if (dr == DrainResult::Stalled || dr == DrainResult::SensorLost) break;

        if (dr == DrainResult::Reached && stepTarget > EMPTY_BAND_MM) {
            trimToLevel(dev, containerZero, floaterThickness, sol_pwm, stepTarget);
        }

        capture100Readings(dev, "FullCycle_PostDrainStep", containerZero, floaterThickness);
        std::cout << "\n";
    }
    setSolenoid(false);
    std::cout << "\n[SYSTEM] Cycle complete. Hardware parked.\n";
}

// ==============================================================================
// SUBMENUS
// ==============================================================================

void menuHardware(Dev_t dev, double& containerZero, double& floaterThickness) {
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
            case 1: runCalibration(dev, containerZero, floaterThickness); break;
            case 2: runSolenoidTestOnly(); break;
            case 3: runManualHardwareControl(); break;
            case 4: runContinuousRead(dev, containerZero, floaterThickness); break;
            case 5: return;
        }
    }
}

void menuFillTests(Dev_t dev) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- FILL TESTS ---\n";
        std::cout << " [1] Tank Zero Distance Experiment (Fill & Monitor + Iterations)\n";
        std::cout << " [2] Back to Main Menu\nSelection: ";

        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runTankZeroExperiment(dev); break;
            case 2: return;
        }
    }
}

void menuDrainTests(Dev_t dev, double containerZero, double floaterThickness) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- DRAIN TESTS ---\n";
        std::cout << " [1] Gravity Drain & Monitor (Solenoid + ToF)\n";
        std::cout << " [2] Back to Main Menu\nSelection: ";

        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runSolenoidAndToF(dev, containerZero, floaterThickness); break;
            case 2: return;
        }
    }
}

void menuFillAndDrainTests(Dev_t dev, double containerZero, double floaterThickness) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- FILL AND DRAIN TESTS ---\n";
        std::cout << " [1] Full Cycle (Pump Fill -> Settle -> Stepped Drain)\n";
        std::cout << " [2] Back to Main Menu\nSelection: ";

        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runFullFluidCycle(dev, containerZero, floaterThickness); break;
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

    if (VL53L4CD_PlatformInit() != 0) {
        std::cerr << "I2C Initialization Failed.\n";
        return 2;
    }

    Dev_t dev = 0;

    uint8_t status = VL53L4CD_SensorInit(dev);
    if (status != 0) {
        std::cerr << "VL53L4CD Sensor Init Failed with error code: " << (int)status << "\n";
        return 2;
    }

    // Accurate timing by default; drainToLevel() temporarily switches to fast timing.
    // (No SetOffset: a constant offset cancels in level = zero - raw - floater.)
    VL53L4CD_SetRangeTiming(dev, TIMING_ACCURATE_MS, 0);
    VL53L4CD_StartRanging(dev);

    currentSessionID = generateSessionID();

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
            case 1: menuHardware(dev, containerZero, floaterThickness); break;
            case 2: menuFillTests(dev); break;
            case 3: menuDrainTests(dev, containerZero, floaterThickness); break;
            case 4: menuFillAndDrainTests(dev, containerZero, floaterThickness); break;
            case 5: systemOffline = 1; break;
        }
    }

    VL53L4CD_StopRanging(dev);

    setPump(false);
    setSolenoid(false);
    std::cout << "\nSystem Offline. Hardware safely parked.\n";
    return 0;
}
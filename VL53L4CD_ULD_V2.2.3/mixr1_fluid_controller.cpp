// mixr1_fluid_controller.cpp  (on/off bang-bang revision)
//
//   FILL TESTS      : pump ONLY. On/Off closed-loop (on predicted error) to a ToF level.
//   DRAIN TESTS     : you fill manually, solenoid ONLY drains to the target with On/Off closed-loop.
//   FILL+DRAIN TESTS: both actuators cooperate (never at the same time) to track a setpoint sequence.
//   Auto re-zero after every drain-to-empty; multi-level calibration run fits ToF->physical correction.
//   Every control run is logged to control_log.csv (time, level, target, error, PWM) for step-response plots.

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

extern "C" {
#include "VL53L4CD_ULD_Driver/VL53L4CD_api.h"
#include "Platform/platform.h"

// Custom platform init (defined in Platform/platform.c)
uint8_t VL53L4CD_PlatformInit(void);
}

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

const char* CALIBRATION_FILE      = "container_zero.txt";
const char* CORRECTION_FILE       = "level_correction.txt";
const char* DATA_FILE             = "fluid_dynamics_data.csv";
const char* RAW_DATA_FILE         = "raw_sensor_data.csv";
const char* REZERO_LOG_FILE       = "rezero_log.csv";
const char* CAL_FIT_FILE          = "calibration_fit.csv";

std::string currentSessionID;

// ---- Sensor timing ----
constexpr uint32_t TIMING_ACCURATE_MS = 200;
constexpr uint32_t TIMING_FAST_MS     = 50;
constexpr double   OUTLIER_BAND_MM    = 3.0;

// ---- Drain tuning ----
constexpr double EMPTY_BAND_MM         = 1.5;
constexpr int    EMPTY_CONFIRM_READS   = 5;
constexpr double FLOOR_MAX_MM          = 10.0;
constexpr double FLOOR_STALL_S         = 3.0;
constexpr double FINAL_DRAIN_DWELL_S   = 6.0;
constexpr double DRAIN_LATENCY_S       = 0.35;
constexpr double RATE_WINDOW_S         = 1.0;
constexpr double DRAIN_STALL_TIMEOUT_S = 20.0;

// ---- Fill tuning ----
constexpr double FILL_UNDERSHOOT_MM    = 3.0;   
constexpr double FILL_LATENCY_S        = 0.55;  // measured: ~3.8 mm keeps arriving after pump-off at 6.8 mm/s
constexpr double FILL_STALL_TIMEOUT_S  = 25.0;  // generous: floater doesn't rise until fluid passes its draft

// ---- Trim tuning ----
constexpr double TARGET_TOLERANCE_MM   = 1.0;
constexpr int    MAX_TRIM_PULSES       = 10;
constexpr int    PUMP_TRIM_MS_PER_MM   = 60;    
constexpr int    SOL_TRIM_MS_PER_MM    = 100;   
constexpr int    TRIM_MIN_MS           = 60;
constexpr int    TRIM_MAX_MS           = 600;
constexpr int    TRIM_SETTLE_US        = 2000000;
constexpr int    PUMP_DEADTIME_MS      = 800;   
constexpr int    PUMP_DEADTIME_STEP_MS = 500;   
constexpr int    PUMP_TRIM_MAX_MS      = 4000;

// ---- Auto re-zero ----
constexpr int    REZERO_SAMPLES        = 30;
constexpr double REZERO_MAX_SHIFT_MM   = 8.0;   

struct SensorMetrics {
    double   average;
    double   median;
    uint16_t min;
    uint16_t max;
    int      validSamples;
    int      targetSamples;
};

enum class MoveResult { Reached, Aborted, Stalled, SensorLost, Unreachable, Timeout };

// ------------------------------------------------------------------------------
// LEVEL CORRECTION (raw ToF level -> physical level)
// ------------------------------------------------------------------------------

struct LevelCorrection {
    double gain = 1.0;                                   
    std::vector<std::pair<double, double>> table;        
};
LevelCorrection g_corr;

double applyCorrection(double x) {
    if (g_corr.table.empty()) return x * g_corr.gain;

    std::vector<std::pair<double, double>> p;
    p.push_back({0.0, 0.0});
    for (const auto& q : g_corr.table) if (q.first > 0.0) p.push_back(q);
    if (p.size() < 2) return x * g_corr.gain;

    if (x <= 0.0) return x * (p[1].second / p[1].first);

    size_t i = 1;
    while (i + 1 < p.size() && x > p[i].first) ++i;
    double dx = p[i].first - p[i - 1].first;
    if (dx <= 0.0) return x * g_corr.gain;
    double t = (x - p[i - 1].first) / dx;                
    return p[i - 1].second + t * (p[i].second - p[i - 1].second);
}

bool saveCorrection() {
    std::ofstream f(CORRECTION_FILE, std::ios::trunc);
    if (!f.is_open()) return false;
    f << std::fixed << std::setprecision(5) << "gain " << g_corr.gain << "\n";
    for (const auto& q : g_corr.table) f << "pt " << q.first << " " << q.second << "\n";
    return static_cast<bool>(f);
}

void loadCorrection() {
    g_corr = LevelCorrection();
    std::ifstream f(CORRECTION_FILE);
    if (!f.is_open()) return;
    std::string key;
    while (f >> key) {
        if (key == "gain") { f >> g_corr.gain; }
        else if (key == "pt") { double x, y; f >> x >> y; g_corr.table.push_back({x, y}); }
        else break;
    }
    std::sort(g_corr.table.begin(), g_corr.table.end());
}

// Single source of truth for fluid level
double computeFluidLevel(double zeroReference_mm, double rawDistance_mm, double floaterThickness_mm, bool clampZero = true) {
    double level = applyCorrection(zeroReference_mm - rawDistance_mm - floaterThickness_mm);
    return clampZero ? std::max(0.0, level) : level;
}

// ------------------------------------------------------------------------------
// HARDWARE PRIMITIVES
// ------------------------------------------------------------------------------

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

std::string nowString() {
    std::time_t t = std::time(nullptr);
    char timeBuf[20];
    std::strftime(timeBuf, sizeof(timeBuf), "%Y-%m-%d %H:%M:%S", std::localtime(&t));
    return std::string(timeBuf);
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

    double error = actualMeasured - calculatedLevel;

    file << sessionID << ","
         << nowString() << ","
         << targetLevel << ","
         << std::fixed << std::setprecision(2) << calculatedLevel << ","
         << actualMeasured << ","
         << rawSensorAvg << ","
         << error << "\n";
}

// ------------------------------------------------------------------------------
// ST API WRAPPERS
// ------------------------------------------------------------------------------

static void discardStale(Dev_t dev) {
    VL53L4CD_ClearInterrupt(dev);
    uint8_t ready = 0;
    while (!ready && !emergencyStop) {
        VL53L4CD_CheckForDataReady(dev, &ready);
        usleep(1000);
    }
    VL53L4CD_ClearInterrupt(dev);
}

void setTiming(Dev_t dev, uint32_t budget_ms) {
    VL53L4CD_StopRanging(dev);
    VL53L4CD_SetRangeTiming(dev, budget_ms, 0);
    VL53L4CD_StartRanging(dev);
}

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

        std::cout << "\rSample " << i << "/100: Raw " << rawDist << " mm | Lvl " << std::fixed << std::setprecision(1) << calculatedLevel << " mm    " << std::flush;
        rawFile << currentSessionID << "," << nowString() << "," << eventName << "," << i << "," << rawDist << "," << std::fixed << std::setprecision(2) << calculatedLevel << "\n";
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
// AUTO RE-ZERO
// ------------------------------------------------------------------------------

// Call ONLY when the tank is confirmed empty (right after a drain-to-empty).
// Re-measures the resting-floater distance and updates the zero reference so
// that an empty tank reads 0.0 mm. 'zero' is updated in place.
bool autoRezero(Dev_t dev, double& zero, double floater) {
    usleep(1500000);   // let the floater come to rest
    SensorMetrics m = getSensorMetrics(dev, REZERO_SAMPLES, 0, true);
    if (m.validSamples < REZERO_SAMPLES / 2) {
        std::cout << "[REZERO] Skipped: not enough valid readings.\n";
        return false;
    }

    double oldRest = zero - floater;
    double newRest = m.average;
    double shift = newRest - oldRest;

    if (std::fabs(shift) > REZERO_MAX_SHIFT_MM) {
        std::cout << "[REZERO] Skipped: resting distance moved " << std::fixed << std::setprecision(2)
                  << shift << " mm (> " << REZERO_MAX_SHIFT_MM << "). Tank may not be empty or floater is tilted.\n";
        return false;
    }

    zero = newRest + floater;

    bool exists = fileExists(REZERO_LOG_FILE);
    std::ofstream f(REZERO_LOG_FILE, std::ios::app);
    if (!exists) f << "SessionID,Timestamp,OldRest_mm,NewRest_mm,Shift_mm\n";
    f << currentSessionID << "," << nowString() << "," << std::fixed << std::setprecision(2)
      << oldRest << "," << newRest << "," << shift << "\n";

    std::cout << "[REZERO] Resting distance " << std::fixed << std::setprecision(2) << oldRest
              << " -> " << newRest << " mm (shift " << shift << " mm). Zero updated.\n";
    return true;
}

// ------------------------------------------------------------------------------
// FILL / DRAIN CONTROL
// ------------------------------------------------------------------------------

void reportMove(MoveResult r, const char* what) {
    switch (r) {
        case MoveResult::Reached:    std::cout << "\n[SYSTEM] " << what << " target reached. Hardware stopped.\n"; break;
        case MoveResult::Aborted:    std::cout << "\n[SYSTEM] " << what << " stopped by user.\n"; break;
        case MoveResult::Stalled:    std::cout << "\n[!] " << what << ": no level change for too long (dry pump / clog / air lock?). Hardware stopped.\n"; break;
        case MoveResult::SensorLost: std::cout << "\n[!] " << what << ": lost valid ToF readings. Hardware stopped.\n"; break;
        case MoveResult::Unreachable: std::cout << "\n[!] " << what << ": settled outside tolerance and the allowed actuator can't correct it.\n"; break;
        case MoveResult::Timeout:    std::cout << "\n[!] " << what << ": timed out. Hardware stopped.\n"; break;
    }
}

// Drain until level <= targetLevel. targetLevel <= EMPTY_BAND_MM => drain-to-empty
// (waits for floater to land, then final dwell so fluid under the floater drains).
MoveResult drainToLevel(Dev_t dev, double zero, double floater, int solPwm,
                        double targetLevel, char* abortKey = nullptr) {
    using clock = std::chrono::steady_clock;
    auto secs = [](clock::time_point a, clock::time_point b) {
        return std::chrono::duration<double>(b - a).count();
    };

    const bool toEmpty = targetLevel <= EMPTY_BAND_MM;
    MoveResult result = MoveResult::Reached;
    {
        FastTimingScope fast(dev);
        while (kbhit()) getchar();
        setSolenoid(true, solPwm);

        const auto t0 = clock::now();
        auto tProgress = t0;
        double bestLevel = 1e9;
        std::deque<std::pair<double, double>> hist;
        int emptyCount = 0, badReads = 0;

        while (!emergencyStop) {
            if (kbhit()) {
                char c = getchar();
                if (abortKey) *abortKey = c;
                result = MoveResult::Aborted;
                break;
            }

            double level;
            if (!readLevel(dev, zero, floater, level)) {
                if (++badReads > 20) { result = MoveResult::SensorLost; break; }
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
                if (stall > FLOOR_STALL_S && level <= FLOOR_MAX_MM) break;
            } else {
                double predicted = level + std::min(rate, 0.0) * DRAIN_LATENCY_S;
                if (predicted <= targetLevel) break;
            }

            if (stall > DRAIN_STALL_TIMEOUT_S) { result = MoveResult::Stalled; break; }
        }

        if (toEmpty && result == MoveResult::Reached && !emergencyStop) {
            std::cout << "\n[SYSTEM] Floor reached. Final drain dwell (" << FINAL_DRAIN_DWELL_S << " s)...\n";
            auto tEnd = clock::now() + std::chrono::milliseconds(static_cast<int>(FINAL_DRAIN_DWELL_S * 1000));
            while (!emergencyStop && clock::now() < tEnd) {
                if (kbhit()) {
                    char c = getchar();
                    if (abortKey) *abortKey = c;
                    result = MoveResult::Aborted;
                    break;
                }
                usleep(50000);
            }
        }
        setSolenoid(false);
    }
    return result;
}

// ------------------------------------------------------------------------------
// CLOSED-LOOP ON/OFF (BANG-BANG) CONTROLLER
// ------------------------------------------------------------------------------

constexpr double CTRL_DEADBAND_MM   = 0.4;    // predicted error below this => actuators off
constexpr double CTRL_SETTLE_RATE   = 0.4;    // mm/s considered "steady"
constexpr double CTRL_SETTLE_HOLD_S = 2.0;    // in tolerance + steady this long => done
constexpr double CTRL_UNREACH_HOLD_S= 3.0;
constexpr double CTRL_TIMEOUT_S     = 240.0;
constexpr double CTRL_STALL_S       = 25.0;   // actuator on but error not improving
constexpr int    SOL_KICK_MS        = 120;    // full-PWM kick when opening the valve
const char* CONTROL_LOG_FILE = "control_log.csv";

struct ControlOptions {
    bool allowPump  = true;
    bool allowDrain = true;
    int  pumpMaxPwm = 1024;
    int  solMaxPwm  = 1024;
    const char* eventName = "Control";
};

MoveResult controlToLevel(Dev_t dev, double zero, double floater, double target,
                          const ControlOptions& opt, char* abortKey = nullptr, double* finalLevel = nullptr) {
    using clock = std::chrono::steady_clock;
    auto secs = [](clock::time_point a, clock::time_point b) {
        return std::chrono::duration<double>(b - a).count();
    };

    bool logExists = fileExists(CONTROL_LOG_FILE);
    std::ofstream clog(CONTROL_LOG_FILE, std::ios::app);
    if (!logExists) clog << "SessionID,Event,Time_s,Level_mm,Predicted_mm,Target_mm,Error_mm,Actuator,PWM\n";

    MoveResult result = MoveResult::Aborted;
    double level = 0.0;
    
    {
        FastTimingScope fast(dev);
        while (kbhit()) getchar();
        setPump(false);
        setSolenoid(false);

        const auto t0 = clock::now();
        auto tProgress = t0, tBand = t0, tStuck = t0;
        double bestAbsErr = 1e9, lastTs = 0.0;
        std::deque<std::pair<double, double>> hist;
        int badReads = 0;
        bool solOpen = false, inBand = false, stuck = false;

        while (!emergencyStop) {
            if (kbhit()) {
                char c = getchar();
                if (abortKey) *abortKey = c;
                result = MoveResult::Aborted;
                break;
            }

            if (!readLevel(dev, zero, floater, level)) {
                if (++badReads > 20) { result = MoveResult::SensorLost; break; }
                continue;
            }
            badReads = 0;

            auto now = clock::now();
            double ts = secs(t0, now);
            lastTs = ts;

            hist.push_back({ts, level});
            while (hist.size() > 2 && ts - hist.front().first > RATE_WINDOW_S) hist.pop_front();
            double rate = 0.0;
            if (hist.size() >= 3) {
                double w = ts - hist.front().first;
                if (w > 0.3) rate = (level - hist.front().second) / w;
            }

            // Latency prediction prevents overshoot when snapping OFF
            double lat = (rate >= 0.0) ? FILL_LATENCY_S : DRAIN_LATENCY_S;
            double pred = level + rate * lat;
            double e = target - pred;
            double absErr = std::fabs(target - level);

            int act = 0, pwm = 0;
            
            // --- SIMPLE ON/OFF (BANG-BANG) LOGIC ---
            if (e > CTRL_DEADBAND_MM && opt.allowPump) {
                act = +1;
                pwm = opt.pumpMaxPwm; // Strictly ON
            } else if (e < -CTRL_DEADBAND_MM && opt.allowDrain) {
                act = -1;
                pwm = opt.solMaxPwm;  // Strictly ON
            }

            // Hardware actuation
            if (act == +1) {
                if (solOpen) { setSolenoid(false); solOpen = false; }
                setPump(true, pwm);
            } else if (act == -1) {
                setPump(false);
                if (!solOpen) {
                    setSolenoid(true, opt.solMaxPwm); // Kick to pull valve in
                    usleep(SOL_KICK_MS * 1000);
                    solOpen = true;
                }
                setSolenoid(true, pwm);
            } else {
                setPump(false);
                if (solOpen) { setSolenoid(false); solOpen = false; }
            }

            std::cout << "\rLvl: " << std::fixed << std::setprecision(1) << level
                      << " mm | Pred: " << pred << " | Target: " << target
                      << " | " << (act > 0 ? "PUMP " : act < 0 ? "DRAIN" : "OFF  ")
                      << " " << std::setw(3) << (pwm * 100 / 1024) << "%    " << std::flush;
            
            clog << currentSessionID << "," << opt.eventName << "," << std::fixed << std::setprecision(3) << ts << ","
                 << std::setprecision(2) << level << "," << pred << "," << target << "," << (target - level) << ","
                 << (act > 0 ? "P" : act < 0 ? "D" : "-") << "," << pwm << "\n";

            // Termination conditions
            bool inTol  = absErr <= TARGET_TOLERANCE_MM;
            bool steady = std::fabs(rate) <= CTRL_SETTLE_RATE;

            if (inTol && steady) {
                if (!inBand) { inBand = true; tBand = now; }
                if (secs(tBand, now) >= CTRL_SETTLE_HOLD_S) { result = MoveResult::Reached; break; }
            } else {
                inBand = false;
            }

            if (act == 0 && !inTol && steady) {
                if (!stuck) { stuck = true; tStuck = now; }
                if (secs(tStuck, now) >= CTRL_UNREACH_HOLD_S) { result = MoveResult::Unreachable; break; }
            } else {
                stuck = false;
            }

            if (absErr < bestAbsErr - 0.5) { bestAbsErr = absErr; tProgress = now; }
            if (act != 0 && secs(tProgress, now) > CTRL_STALL_S) { result = MoveResult::Stalled; break; }
            if (ts > CTRL_TIMEOUT_S) { result = MoveResult::Timeout; break; }
        }
        
        setPump(false);
        setSolenoid(false);
    }
    
    if (finalLevel) *finalLevel = level;
    return result;
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
            double rawLevel = computeFluidLevel(containerZero, metrics.average, floaterThickness, false);
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
    std::cout << ">> Calculated Floater Thickness: " << std::fixed << std::setprecision(2) << floaterThickness << " mm\n";
    if (floaterThickness < 0.0) {
        std::cout << "[!] Negative thickness: your tank-bottom distance is SMALLER than the floater resting distance. Re-measure it.\n";
    }
    std::cout << "\n";

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

        std::cout << "Filling (On/Off closed-loop, PUMP ONLY)... Press ANY KEY to stop, or 'q' to EXIT experiment.\n";

        char key = 0;
        ControlOptions co;
        co.allowPump = true;
        co.allowDrain = false;
        co.pumpMaxPwm = pump_pwm;
        co.solMaxPwm = sol_pwm;
        co.eventName = "TankZeroFill";
        MoveResult fr = controlToLevel(dev, tankBottom, floaterThickness, targetLevel, co, &key);
        reportMove(fr, "Fill");
        if (fr == MoveResult::Aborted && (key == 'q' || key == 'Q')) abortExp = true;
        if (abortExp || emergencyStop || fr == MoveResult::Stalled || fr == MoveResult::SensorLost) {
            std::cout << "\n[ABORT] Exiting experiment...\n";
            break;
        }

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
        key = 0;
        MoveResult dr = drainToLevel(dev, tankBottom, floaterThickness, sol_pwm, 0.0, &key);
        if (dr == MoveResult::Aborted && (key == 'q' || key == 'Q')) abortExp = true;
        reportMove(dr, "Drain");
        if (abortExp || dr == MoveResult::Stalled || dr == MoveResult::SensorLost) break;

        if (dr == MoveResult::Reached) {
            autoRezero(dev, tankBottom, floaterThickness);   // empty tank must read 0.0
        }

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

// Drain test: you fill manually, the solenoid (on/off, closed-loop) drains to the target.
void runControlledDrain(Dev_t dev, double& containerZero, double& floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }

    std::cout << "\n--- [ CONTROLLED DRAIN TO TARGET (MANUAL FILL) ] ---\n";
    std::cout << "Fill the tank by hand (floater IN). The solenoid drains it to your target using On/Off feedback.\n";
    int sol_pwm = getPWMFromVoltage("Solenoid");

    double target = 0.0;
    std::cout << "Enter target level (mm): ";
    if (!(std::cin >> target) || target <= 0.0) {
        std::cout << "[!] Invalid target.\n";
        std::cin.clear(); std::cin.ignore(10000, '\n'); return;
    }
    std::cin.ignore(10000, '\n');

    std::cout << "Fill the tank above the target, let it settle, then press ENTER (or 'q' to abort)... ";
    std::string dummy;
    std::cin.clear();
    std::getline(std::cin, dummy);
    if (dummy == "q" || dummy == "Q") return;

    SensorMetrics start = getSensorMetrics(dev, 10, 0, true);
    if (start.validSamples == 0) { std::cout << "[!] No valid ToF readings.\n"; return; }
    double startLevel = computeFluidLevel(containerZero, start.average, floaterThickness, false);
    std::cout << "Start level: " << std::fixed << std::setprecision(1) << startLevel << " mm\n";
    if (startLevel <= target + TARGET_TOLERANCE_MM) {
        std::cout << "[!] Tank is already at or below the target. Nothing to drain.\n";
        return;
    }

    emergencyStop = 0;
    std::cout << "Draining (On/Off closed-loop, SOLENOID ONLY)... Press ANY KEY to abort.\n";
    ControlOptions co;
    co.allowPump = false;
    co.allowDrain = true;
    co.solMaxPwm = sol_pwm;
    co.eventName = "ControlledDrain";
    MoveResult r = controlToLevel(dev, containerZero, floaterThickness, target, co);
    reportMove(r, "Drain");

    SensorMetrics fin = getSensorMetrics(dev, 30, 0, true);
    if (fin.validSamples == 0) return;
    double calc = computeFluidLevel(containerZero, fin.average, floaterThickness);
    std::cout << "Final ToF level: " << std::fixed << std::setprecision(2) << calc
              << " mm (target " << target << ", error " << (calc - target) << ")\n";
    std::cout << "Enter actual measured level (mm) [-1 to skip]: ";
    double actual = -1.0;
    if (std::cin >> actual) {
        std::cin.ignore(10000, '\n');
        if (actual >= 0.0) logCycleData(currentSessionID, static_cast<int>(target), calc, actual, fin.average);
    } else {
        std::cin.clear(); std::cin.ignore(10000, '\n');
    }
}

void runDrainToEmpty(Dev_t dev, double& containerZero, double& floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }

    std::cout << "\n--- [ DRAIN TO EMPTY ] ---\n";
    int sol_pwm = getPWMFromVoltage("Solenoid");

    std::cout << "Press ENTER to OPEN SOLENOID and drain (or type 'q' to abort)... ";
    std::string dummy;
    std::cin.clear();
    std::getline(std::cin, dummy);
    if (dummy == "q" || dummy == "Q") return;

    emergencyStop = 0;
    std::cout << "Draining... Press ANY KEY to abort.\n";
    MoveResult dr = drainToLevel(dev, containerZero, floaterThickness, sol_pwm, 0.0);
    reportMove(dr, "Drain");

    if (dr == MoveResult::Reached) {
        if (autoRezero(dev, containerZero, floaterThickness)) saveCalibration(containerZero, floaterThickness);
    }
}

// ==============================================================================
// FILL AND DRAIN TESTS
// ==============================================================================

void runFullFluidCycle(Dev_t dev, double& containerZero, double& floaterThickness) {
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

    std::cout << "Filling (On/Off closed-loop, PUMP ONLY)... Press ANY KEY to abort.\n";
    ControlOptions fillOpt;
    fillOpt.allowPump = true;
    fillOpt.allowDrain = false;
    fillOpt.pumpMaxPwm = pump_pwm;
    fillOpt.solMaxPwm = sol_pwm;
    fillOpt.eventName = "FullCycleFill";
    MoveResult fr = controlToLevel(dev, containerZero, floaterThickness, targetLevel, fillOpt);
    reportMove(fr, "Fill");
    if (fr == MoveResult::Aborted || fr == MoveResult::Stalled || fr == MoveResult::SensorLost) return;
    if (emergencyStop) return;

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
        MoveResult dr;
        if (stepTarget > EMPTY_BAND_MM) {
            ControlOptions stepOpt;
            stepOpt.allowPump = false;
            stepOpt.allowDrain = true;
            stepOpt.solMaxPwm = sol_pwm;
            stepOpt.eventName = "FullCycleStepDrain";
            dr = controlToLevel(dev, containerZero, floaterThickness, stepTarget, stepOpt);
            reportMove(dr, "Drain");
        } else {
            dr = drainToLevel(dev, containerZero, floaterThickness, sol_pwm, 0.0);
            reportMove(dr, "Drain");
            if (dr == MoveResult::Reached) {
                if (autoRezero(dev, containerZero, floaterThickness)) saveCalibration(containerZero, floaterThickness);
            }
        }
        if (dr == MoveResult::Stalled || dr == MoveResult::SensorLost) break;

        capture100Readings(dev, "FullCycle_PostDrainStep", containerZero, floaterThickness);
        std::cout << "\n";
    }
    setSolenoid(false);
    std::cout << "\n[SYSTEM] Cycle complete. Hardware parked.\n";
}

// ------------------------------------------------------------------------------
// MULTI-LEVEL CALIBRATION RUN
// Fills to several levels, you enter physical measurements, then it fits the
// correction (ToF level -> physical level) and saves it.
// ------------------------------------------------------------------------------

void runLevelCalibrationRun(Dev_t dev, double& containerZero, double& floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }

    std::cout << "\n--- [ MULTI-LEVEL CALIBRATION RUN ] ---\n";
    std::cout << "Start with the tank EMPTY and the FLOATER IN.\n";
    std::cout << "Measure the physical level the SAME WAY every time.\n\n";
    std::cout << "Enter levels in mm, ascending, separated by spaces (e.g. 20 40 60 80 100): ";

    std::string line;
    std::cin.clear();
    std::getline(std::cin, line);
    std::istringstream iss(line);
    std::vector<double> levels;
    double v;
    while (iss >> v) if (v > 0.0) levels.push_back(v);
    if (levels.size() < 2) {
        std::cout << "[!] Need at least 2 levels.\n";
        return;
    }
    std::sort(levels.begin(), levels.end());

    int perLevel = 3;
    std::cout << "Physical measurements per level (average is used): ";
    if (!(std::cin >> perLevel) || perLevel <= 0) { std::cin.clear(); perLevel = 3; }
    std::cin.ignore(10000, '\n');

    int pump_pwm = getPWMFromVoltage("Pump");
    int sol_pwm = getPWMFromVoltage("Solenoid");

    // Run with identity correction so we record RAW ToF levels.
    LevelCorrection savedCorr = g_corr;
    g_corr = LevelCorrection();

    emergencyStop = 0;
    std::vector<std::pair<double, double>> pts;   // (toF level, physical)
    bool abortRun = false;

    bool exists = fileExists(CAL_FIT_FILE);
    std::ofstream fitFile(CAL_FIT_FILE, std::ios::app);
    if (!exists) fitFile << "SessionID,Timestamp,TargetLevel_mm,ToFLevel_mm,ToFRawAvg_mm,Physical_mm\n";

    for (size_t li = 0; li < levels.size() && !abortRun && !emergencyStop; ++li) {
        double target = levels[li];
        std::cout << "\n=== LEVEL " << (li + 1) << "/" << levels.size() << ": " << target << " mm ===\n";
        std::cout << "Make sure the FLOATER is IN. Press ENTER to fill (or 'q' to stop run)... ";
        std::string in;
        std::getline(std::cin, in);
        if (in == "q" || in == "Q") break;

        char key = 0;
        ControlOptions calOpt;
        calOpt.allowPump = true;
        calOpt.allowDrain = false;
        calOpt.pumpMaxPwm = pump_pwm;
        calOpt.solMaxPwm = sol_pwm;
        calOpt.eventName = "CalRunFill";
        MoveResult fr = controlToLevel(dev, containerZero, floaterThickness, target, calOpt, &key);
        reportMove(fr, "Fill");
        if (fr == MoveResult::Aborted || fr == MoveResult::Stalled || fr == MoveResult::SensorLost) break;
        if (emergencyStop) break;

        SensorMetrics m = getSensorMetrics(dev, 30, 0, true);
        if (m.validSamples < 10) { std::cout << "[!] Not enough valid readings. Stopping run.\n"; break; }
        double tofLevel = computeFluidLevel(containerZero, m.average, floaterThickness, false);
        std::cout << "[CAL] ToF level: " << std::fixed << std::setprecision(2) << tofLevel << " mm\n";

        std::cout << "Take your physical measurements now (remove floater if that is your method).\n";
        double sum = 0.0;
        int got = 0;
        for (int p = 1; p <= perLevel; ++p) {
            double pm;
            std::cout << "  Physical measurement #" << p << " (mm) [-1 to stop run]: ";
            if (!(std::cin >> pm)) { std::cin.clear(); std::cin.ignore(10000, '\n'); --p; continue; }
            std::cin.ignore(10000, '\n');
            if (pm < 0.0) { abortRun = true; break; }
            sum += pm; ++got;
        }
        if (got > 0) {
            double phys = sum / got;
            pts.push_back({tofLevel, phys});
            fitFile << currentSessionID << "," << nowString() << "," << std::fixed << std::setprecision(2)
                    << target << "," << tofLevel << "," << m.average << "," << phys << "\n";
            std::cout << "[CAL] Recorded: ToF " << tofLevel << " mm  <->  physical " << phys << " mm\n";
        }
        if (!abortRun) {
            std::cout << "Put the FLOATER back in the tank, then press ENTER to continue... ";
            std::getline(std::cin, in);
        }
    }

    // Drain and re-zero
    if (!emergencyStop) {
        std::cout << "\nPress ENTER to drain the tank (floater IN)... ";
        std::string in;
        std::getline(std::cin, in);
        MoveResult dr = drainToLevel(dev, containerZero, floaterThickness, sol_pwm, 0.0);
        reportMove(dr, "Drain");
        if (dr == MoveResult::Reached) {
            if (autoRezero(dev, containerZero, floaterThickness)) saveCalibration(containerZero, floaterThickness);
        }
    }

    // ---- Fit ----
    if (pts.size() < 2) {
        std::cout << "[CAL] Not enough points to fit. Keeping previous correction.\n";
        g_corr = savedCorr;
        return;
    }

    double sxx = 0, sxy = 0, sx = 0, sy = 0, syy = 0;
    const double n = static_cast<double>(pts.size());
    for (auto& p : pts) { sxx += p.first * p.first; sxy += p.first * p.second; sx += p.first; sy += p.second; syy += p.second * p.second; }
    double gain0 = sxy / sxx;                                    // through origin
    double denom = n * sxx - sx * sx;
    double slope = (denom != 0.0) ? (n * sxy - sx * sy) / denom : 1.0;
    double icpt = (sy - slope * sx) / n;

    std::cout << "\n=========== CALIBRATION RESULT ===========\n";
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  ToF(mm)  Physical(mm)  Err@gain0(mm)\n";
    double ssr = 0.0;
    for (auto& p : pts) {
        double e = p.second - gain0 * p.first;
        ssr += e * e;
        std::cout << "  " << std::setw(7) << p.first << "  " << std::setw(12) << p.second << "  " << std::setw(12) << e << "\n";
    }
    std::cout << std::setprecision(4);
    std::cout << "Through-origin gain : " << gain0 << "  (residual RMS " << std::setprecision(2) << std::sqrt(ssr / n) << " mm)\n";
    std::cout << std::setprecision(4);
    std::cout << "Free linear fit     : physical = " << slope << " * ToF + " << std::setprecision(2) << icpt << " mm\n";
    std::cout << "(A large free-fit intercept or curved residuals => use the piecewise table.)\n\n";

    std::cout << "Apply which correction?\n"
              << " [1] Linear gain through origin (recommended: empty stays 0)\n"
              << " [2] Piecewise table through your points\n"
              << " [3] Discard (keep previous)\nSelection: ";
    int sel = 3;
    if (!(std::cin >> sel)) { std::cin.clear(); sel = 3; }
    std::cin.ignore(10000, '\n');

    if (sel == 1) {
        g_corr = LevelCorrection();
        g_corr.gain = gain0;
        saveCorrection();
        std::cout << "[CAL] Saved linear gain " << std::setprecision(4) << gain0 << " to " << CORRECTION_FILE << ".\n";
    } else if (sel == 2) {
        g_corr = LevelCorrection();
        g_corr.gain = gain0;
        g_corr.table = pts;
        std::sort(g_corr.table.begin(), g_corr.table.end());
        saveCorrection();
        std::cout << "[CAL] Saved piecewise table (" << pts.size() << " points) to " << CORRECTION_FILE << ".\n";
    } else {
        g_corr = savedCorr;
        std::cout << "[CAL] Discarded. Previous correction restored.\n";
    }
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

// Fill+Drain test: pump and solenoid cooperate (never together) to track a setpoint sequence.
void runSetpointSequence(Dev_t dev, double& containerZero, double& floaterThickness) {
    if (containerZero == 0.0) {
        std::cout << "[!] Run calibration first.\n";
        return;
    }

    std::cout << "\n--- [ SETPOINT SEQUENCE: PUMP + SOLENOID TOGETHER ] ---\n";
    std::cout << "The controller fills with the pump when below the setpoint and drains with the solenoid when above.\n";
    std::cout << "Enter setpoints in mm separated by spaces (e.g. 50 20 70 30): ";

    std::string line;
    std::cin.clear();
    std::getline(std::cin, line);
    std::istringstream iss(line);
    std::vector<double> sp;
    double v;
    while (iss >> v) if (v > 0.0) sp.push_back(v);
    if (sp.empty()) { std::cout << "[!] No valid setpoints.\n"; return; }

    int pump_pwm = getPWMFromVoltage("Pump");
    int sol_pwm = getPWMFromVoltage("Solenoid");

    std::cout << "Floater IN the tank. Press ENTER to start (or 'q' to abort)... ";
    std::string dummy;
    std::getline(std::cin, dummy);
    if (dummy == "q" || dummy == "Q") return;

    emergencyStop = 0;
    for (size_t i = 0; i < sp.size() && !emergencyStop; ++i) {
        std::cout << "\n=== SETPOINT " << (i + 1) << "/" << sp.size() << ": " << sp[i] << " mm ===\n";
        ControlOptions co;
        co.allowPump = true;
        co.allowDrain = true;
        co.pumpMaxPwm = pump_pwm;
        co.solMaxPwm = sol_pwm;
        co.eventName = "Setpoint";
        MoveResult r = controlToLevel(dev, containerZero, floaterThickness, sp[i], co);
        reportMove(r, "Control");
        if (r == MoveResult::Aborted || r == MoveResult::Stalled || r == MoveResult::SensorLost) break;

        SensorMetrics fin = getSensorMetrics(dev, 30, 0, true);
        if (fin.validSamples == 0) continue;
        double calc = computeFluidLevel(containerZero, fin.average, floaterThickness);
        std::cout << "Settled ToF level: " << std::fixed << std::setprecision(2) << calc
                  << " mm (setpoint " << sp[i] << ", error " << (calc - sp[i]) << ")\n";
        std::cout << "Enter actual measured level (mm) [-1 to skip]: ";
        double actual = -1.0;
        if (std::cin >> actual) {
            std::cin.ignore(10000, '\n');
            if (actual >= 0.0) logCycleData(currentSessionID, static_cast<int>(sp[i]), calc, actual, fin.average);
        } else {
            std::cin.clear(); std::cin.ignore(10000, '\n');
        }
    }
    setPump(false);
    setSolenoid(false);
    std::cout << "\n[SYSTEM] Sequence finished. Hardware parked.\n";
}

void menuDrainTests(Dev_t dev, double& containerZero, double& floaterThickness) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- DRAIN TESTS ---\n";
        std::cout << " [1] Controlled Drain to Target (manual fill, solenoid On/Off control)\n";
        std::cout << " [2] Drain to Empty (+ auto re-zero)\n";
        std::cout << " [3] Back to Main Menu\nSelection: ";

        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runControlledDrain(dev, containerZero, floaterThickness); break;
            case 2: runDrainToEmpty(dev, containerZero, floaterThickness); break;
            case 3: return;
        }
    }
}

void menuFillAndDrainTests(Dev_t dev, double& containerZero, double& floaterThickness) {
    int choice = 0;
    while (!systemOffline) {
        std::cout << "\n--- FILL AND DRAIN TESTS ---\n";
        std::cout << " [1] Setpoint Sequence (pump + solenoid cooperate, On/Off control)\n";
        std::cout << " [2] Full Cycle (Pump Fill -> Settle -> Stepped Drain)\n";
        std::cout << " [3] Multi-Level Calibration Run (fit ToF -> physical correction)\n";
        std::cout << " [4] Back to Main Menu\nSelection: ";

        if (!(std::cin >> choice)) { std::cin.clear(); std::cin.ignore(10000, '\n'); continue; }
        std::cin.ignore(10000, '\n');

        switch (choice) {
            case 1: runSetpointSequence(dev, containerZero, floaterThickness); break;
            case 2: runFullFluidCycle(dev, containerZero, floaterThickness); break;
            case 3: runLevelCalibrationRun(dev, containerZero, floaterThickness); break;
            case 4: return;
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

    VL53L4CD_SetRangeTiming(dev, TIMING_ACCURATE_MS, 0);
    VL53L4CD_StartRanging(dev);

    currentSessionID = generateSessionID();

    double containerZero = 0.0;
    double floaterThickness = 0.0;
    loadCalibration(containerZero, floaterThickness);
    loadCorrection();

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
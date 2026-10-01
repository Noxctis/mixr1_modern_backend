// mixr1_fluid_controller.cpp
//
// MIXR-1 fluid dynamics controller
//   Raspberry Pi 4B, VL53L4CD ToF sensor (ST ULD driver), 2x VNH5019 (pump + solenoid)
//
// Build (adjust paths to your Makefile):
//   g++ -std=c++14 -O2 -Wall -Wextra mixr1_fluid_controller.cpp <ST driver + platform .c/.cpp> -lwiringPi
//
// Sensor offset
//   The offset comes from tof_offset.txt (written by tof_cal) so the controller always
//   uses the same value as your calibration tool. If that file is missing, the default
//   kDefaultOffsetMm (-26 mm) is applied. NOTE: VL53L4CD_SetOffset() takes MILLIMETRES;
//   the driver does the x4 internally. Do not multiply by 4 yourself.
//
// container_zero.txt now also records the offset that was active when the zero was
// taken ("zero thickness offset"). The old 2-column format still loads, but you get a
// warning because the offset used for that zero is unknown.

#include <wiringPi.h>

#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <ctime>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

extern "C" {
#include "VL53L4CD_ULD_Driver/VL53L4CD_api.h"
#include "Platform/platform.h"

// Declared explicitly in case an older platform.h is picked up first
uint8_t VL53L4CD_PlatformInit(void);
}

namespace {

// ============================================================================
// Configuration
// ============================================================================
namespace cfg {

// BCM pins - pump (VNH5019 #1)
constexpr int kPumpIna = 17;
constexpr int kPumpInb = 27;
constexpr int kPumpPwm = 13;  // hardware PWM1
constexpr int kPumpEn  = 15;

// BCM pins - solenoid (VNH5019 #2)
constexpr int kSolenoidIna = 5;
constexpr int kSolenoidInb = 6;
constexpr int kSolenoidPwm = 12;  // hardware PWM0

constexpr int    kPwmRange   = 1024;  // wiringPi default PWM range
constexpr double kMinVoltage = 6.0;
constexpr double kMaxVoltage = 12.0;

// Sensor
constexpr Dev_t    kSensorDev       = 0x52;  // same value as tof_cal; platform layer ignores it
constexpr uint16_t kSensorId        = 0xEBAA;
constexpr uint16_t kTimingBudgetMs  = 200;   // MUST match tof_cal or the calibration is invalid
constexpr int16_t  kDefaultOffsetMm = -26;   // used only if tof_offset.txt is missing
constexpr int      kMaxSaneOffsetMm = 2000;
constexpr int      kWarmupFrames    = 5;
constexpr int      kFrameTimeoutMs  = 2000;
constexpr uint16_t kMaxDistanceMm   = 2000;
constexpr int      kMaxFaultCycles  = 10;    // consecutive reads with no valid sample => stop

// Level logic
constexpr double kLevelGainCorrection = 1.0;
constexpr double kEmptyLevelMm        = 2.0;
constexpr int    kRawCaptureCount     = 100;

// Files
constexpr const char* kOffsetFile      = "tof_offset.txt";
constexpr const char* kCalibrationFile = "container_zero.txt";
constexpr const char* kDataFile        = "fluid_dynamics_data.csv";
constexpr const char* kRawDataFile     = "raw_sensor_data.csv";
constexpr const char* kExperimentFile  = "tank_zero_experiment.csv";
constexpr const char* kLiveDataFile    = "live_level_log.csv";

}  // namespace cfg

// ============================================================================
// Global run state (written from signal handler)
// ============================================================================
volatile std::sig_atomic_t g_emergencyStop = 0;
volatile std::sig_atomic_t g_systemOffline = 0;

// ============================================================================
// Small utilities
// ============================================================================
std::string timestamp(const char* format) {
    std::time_t t = std::time(nullptr);
    std::tm tmBuf;
    localtime_r(&t, &tmBuf);
    char buf[32];
    std::strftime(buf, sizeof(buf), format, &tmBuf);
    return buf;
}

std::string fmt(double v, int precision = 2) {
    std::ostringstream s;
    s << std::fixed << std::setprecision(precision) << v;
    return s.str();
}

void sleepMs(int ms) { usleep(static_cast<useconds_t>(ms) * 1000); }

double median(std::vector<double> v) {
    if (v.empty()) return 0.0;
    std::sort(v.begin(), v.end());
    const size_t n = v.size();
    return (n % 2) ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}

bool isQuit(const std::string& s) { return s == "q" || s == "Q"; }

// ----------------------------------------------------------------------------
// Console input (line based; never mixes operator>> with getline)
// ----------------------------------------------------------------------------
bool readLine(std::string& line) {
    if (!std::getline(std::cin, line)) {
        g_systemOffline = 1;  // stdin closed or interrupted: shut down cleanly
        return false;
    }
    return true;
}

template <typename T>
bool parseNumber(const std::string& s, T& out) {
    std::istringstream iss(s);
    T v;
    if (!(iss >> v)) return false;
    iss >> std::ws;
    if (!iss.eof()) return false;
    out = v;
    return true;
}

template <typename T>
bool promptNumber(const std::string& prompt, T& out,
                  T lo = std::numeric_limits<T>::lowest(),
                  T hi = std::numeric_limits<T>::max()) {
    std::cout << prompt << std::flush;
    std::string line;
    if (!readLine(line)) return false;
    T v;
    if (!parseNumber(line, v) || v < lo || v > hi) {
        std::cout << "[!] Invalid input.\n";
        return false;
    }
    out = v;
    return true;
}

// Waits for ENTER. Returns false if the user typed 'q' or input closed.
bool confirm(const std::string& prompt) {
    std::cout << prompt << std::flush;
    std::string line;
    if (!readLine(line)) return false;
    return !isQuit(line);
}

// ----------------------------------------------------------------------------
// Non-blocking key detection. Terminal is in raw mode only while the object lives.
// ----------------------------------------------------------------------------
class KeyPoller {
public:
    KeyPoller() {
        if (!isatty(STDIN_FILENO) || tcgetattr(STDIN_FILENO, &saved_) != 0) return;
        termios raw = saved_;
        raw.c_lflag &= ~static_cast<tcflag_t>(ICANON | ECHO);
        raw.c_cc[VMIN] = 0;
        raw.c_cc[VTIME] = 0;
        active_ = (tcsetattr(STDIN_FILENO, TCSANOW, &raw) == 0);
    }
    ~KeyPoller() {
        if (!active_) return;
        drain();
        tcsetattr(STDIN_FILENO, TCSANOW, &saved_);
    }
    KeyPoller(const KeyPoller&) = delete;
    KeyPoller& operator=(const KeyPoller&) = delete;

    bool pressed() const {
        if (!active_) return false;
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        timeval tv;
        tv.tv_sec = 0;
        tv.tv_usec = 0;
        return select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0;
    }
    int readKey() const {
        if (!active_) return -1;
        unsigned char c;
        return (::read(STDIN_FILENO, &c, 1) == 1) ? c : -1;
    }
    void drain() const {
        while (pressed() && readKey() >= 0) {}
    }

private:
    termios saved_{};
    bool active_ = false;
};

// ----------------------------------------------------------------------------
// CSV helper: appends, writes header when the file is new/empty
// ----------------------------------------------------------------------------
class CsvFile {
public:
    CsvFile(const std::string& path, const std::string& header) {
        bool fresh;
        {
            std::ifstream f(path);
            fresh = !(f.good() && f.peek() != std::ifstream::traits_type::eof());
        }
        out_.open(path, std::ios::app);
        if (!out_.is_open()) {
            std::cerr << "\n[!] Could not open " << path << " for logging.\n";
            return;
        }
        if (fresh) out_ << header << '\n';
    }
    bool ok() const { return out_.is_open(); }
    std::ofstream& stream() { return out_; }

private:
    std::ofstream out_;
};

// ============================================================================
// Actuators
// ============================================================================
void setSolenoid(bool open, int pwm = cfg::kPwmRange) {
    if (open) {
        digitalWrite(cfg::kSolenoidIna, HIGH);
        digitalWrite(cfg::kSolenoidInb, LOW);
        pwmWrite(cfg::kSolenoidPwm, pwm);
    } else {
        digitalWrite(cfg::kSolenoidIna, LOW);
        digitalWrite(cfg::kSolenoidInb, LOW);
        pwmWrite(cfg::kSolenoidPwm, 0);
    }
}

void setPump(bool active, int pwm = cfg::kPwmRange) {
    if (active) {
        digitalWrite(cfg::kPumpEn, HIGH);
        digitalWrite(cfg::kPumpIna, HIGH);
        digitalWrite(cfg::kPumpInb, LOW);
        pwmWrite(cfg::kPumpPwm, pwm);
    } else {
        digitalWrite(cfg::kPumpEn, LOW);
        digitalWrite(cfg::kPumpIna, LOW);
        digitalWrite(cfg::kPumpInb, LOW);
        pwmWrite(cfg::kPumpPwm, 0);
    }
}

void parkHardware() {
    setPump(false);
    setSolenoid(false);
}

struct Actuator {
    const char* name;
    void (*set)(bool, int);
};
const Actuator kPump     = {"Pump", setPump};
const Actuator kSolenoid = {"Solenoid", setSolenoid};

// Turns an actuator on for the lifetime of the object; guarantees OFF on every exit path.
class ScopedActuator {
public:
    ScopedActuator(const Actuator& a, int pwm) : actuator_(a) { actuator_.set(true, pwm); }
    ~ScopedActuator() { actuator_.set(false, 0); }
    ScopedActuator(const ScopedActuator&) = delete;
    ScopedActuator& operator=(const ScopedActuator&) = delete;

private:
    Actuator actuator_;
};

// Only register writes happen here, which is why it is acceptable in a signal handler.
// No SA_RESTART: a blocked getline() returns so the program can shut down promptly.
void onSignal(int) {
    g_emergencyStop = 1;
    g_systemOffline = 1;
    parkHardware();
}

void installSignalHandlers() {
    struct sigaction sa;
    std::memset(&sa, 0, sizeof(sa));
    sa.sa_handler = onSignal;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sigaction(SIGINT, &sa, nullptr);
    sigaction(SIGTERM, &sa, nullptr);
}

int promptPwm(const std::string& hwName) {
    std::cout << "Enter " << hwName << " operating voltage ("
              << cfg::kMinVoltage << " - " << cfg::kMaxVoltage << " V): " << std::flush;
    std::string line;
    double v = 0.0;
    if (!readLine(line) || !parseNumber(line, v) || v < cfg::kMinVoltage || v > cfg::kMaxVoltage) {
        std::cout << "[!] Invalid voltage. Defaulting to " << cfg::kMaxVoltage << "V.\n";
        return cfg::kPwmRange;
    }
    const int pwm = static_cast<int>((v / cfg::kMaxVoltage) * cfg::kPwmRange);
    return std::max(0, std::min(cfg::kPwmRange, pwm));
}

// ============================================================================
// ToF sensor
// ============================================================================
struct SensorMetrics {
    double median = 0.0;
    double min = 0.0;
    double max = 0.0;
    int validSamples = 0;
    int requestedSamples = 0;
};

class ToFSensor {
public:
    // Same init sequence as tof_cal so calibration carries over.
    bool init(int16_t offsetMm) {
        if (VL53L4CD_PlatformInit() != 0) {
            std::cerr << "I2C initialization failed.\n";
            return false;
        }
        uint16_t id = 0;
        if (VL53L4CD_GetSensorId(cfg::kSensorDev, &id) != 0 || id != cfg::kSensorId) {
            std::cerr << "VL53L4CD not detected (id=0x" << std::hex << id << std::dec << ").\n";
            return false;
        }
        if (VL53L4CD_SensorInit(cfg::kSensorDev) != 0) {
            std::cerr << "VL53L4CD SensorInit failed.\n";
            return false;
        }
        if (VL53L4CD_SetRangeTiming(cfg::kSensorDev, cfg::kTimingBudgetMs, 0) != 0) {
            std::cerr << "VL53L4CD SetRangeTiming failed.\n";
            return false;
        }
        if (VL53L4CD_SetOffset(cfg::kSensorDev, offsetMm) != 0) {  // millimetres, NOT x4
            std::cerr << "VL53L4CD SetOffset failed.\n";
            return false;
        }
        offsetMm_ = offsetMm;
        // Temperature compensation; sensor must not be ranging. Repeat if ambient moves ~8 C.
        VL53L4CD_StartTemperatureUpdate(cfg::kSensorDev);
        return true;
    }

    // Starts ranging and discards the first frames (they are unreliable).
    bool startRanging() {
        if (VL53L4CD_StartRanging(cfg::kSensorDev) != 0) {
            std::cerr << "VL53L4CD StartRanging failed.\n";
            return false;
        }
        uint16_t d;
        uint8_t s;
        for (int i = 0; i < cfg::kWarmupFrames; ++i) {
            if (!readFrame(d, s)) return false;
        }
        return true;
    }

    void stopRanging() { VL53L4CD_StopRanging(cfg::kSensorDev); }

    int offsetMm() const { return offsetMm_; }

    // Blocks until a new frame is ready. Returns false on timeout, bus error or emergency stop.
    bool readFrame(uint16_t& distanceMm, uint8_t& status) {
        uint8_t ready = 0;
        int waitedMs = 0;
        while (!g_emergencyStop) {
            if (VL53L4CD_CheckForDataReady(cfg::kSensorDev, &ready) != 0) return false;
            if (ready) break;
            if (waitedMs >= cfg::kFrameTimeoutMs) return false;
            sleepMs(1);
            ++waitedMs;
        }
        if (!ready) return false;

        VL53L4CD_ResultsData_t r;
        if (VL53L4CD_GetResult(cfg::kSensorDev, &r) != 0) return false;
        VL53L4CD_ClearInterrupt(cfg::kSensorDev);
        distanceMm = r.distance_mm;
        status = r.range_status;
        return true;
    }

    // Collects `frames` frames and returns stats over the valid ones (median, like tof_cal).
    // discardFirst drops one frame, which may be stale if nothing was reading for a while.
    SensorMetrics measure(int frames, bool discardFirst = false) {
        SensorMetrics m;
        m.requestedSamples = frames;
        uint16_t d;
        uint8_t s;
        if (discardFirst) readFrame(d, s);

        std::vector<double> valid;
        valid.reserve(static_cast<size_t>(frames));
        for (int i = 0; i < frames && !g_emergencyStop; ++i) {
            if (!readFrame(d, s)) break;
            if (s == 0 && d > 0 && d < cfg::kMaxDistanceMm) valid.push_back(d);
        }
        if (valid.empty()) return m;

        m.validSamples = static_cast<int>(valid.size());
        m.median = median(valid);
        m.min = *std::min_element(valid.begin(), valid.end());
        m.max = *std::max_element(valid.begin(), valid.end());
        return m;
    }

private:
    int offsetMm_ = 0;
};

// Reads the offset saved by tof_cal; falls back to the built-in default.
int16_t loadSensorOffset() {
    std::ifstream f(cfg::kOffsetFile);
    int v = 0;
    if (f >> v) {
        if (std::abs(v) <= cfg::kMaxSaneOffsetMm) {
            std::cout << "Sensor offset " << v << " mm (from " << cfg::kOffsetFile << ")\n";
            return static_cast<int16_t>(v);
        }
        std::cerr << "[!] " << cfg::kOffsetFile << " holds an implausible offset (" << v
                  << " mm); ignoring it.\n";
    }
    std::cout << "[!] No usable " << cfg::kOffsetFile << " - applying default offset "
              << cfg::kDefaultOffsetMm << " mm\n";
    return cfg::kDefaultOffsetMm;
}

// ============================================================================
// Calibration + level maths
// ============================================================================
struct Calibration {
    double containerZero_mm = 0.0;
    double floaterThickness_mm = 0.0;
    bool hasOffsetRecord = false;  // false for legacy files
    int offsetMm = 0;              // sensor offset active when the zero was taken

    bool valid() const { return containerZero_mm > 0.0; }
};

bool saveCalibration(const Calibration& c) {
    std::ofstream file(cfg::kCalibrationFile, std::ios::trunc);
    if (!file.is_open()) return false;
    file << fmt(c.containerZero_mm) << " " << fmt(c.floaterThickness_mm) << " " << c.offsetMm;
    return static_cast<bool>(file);
}

Calibration loadCalibration() {
    Calibration c;
    std::ifstream file(cfg::kCalibrationFile);
    double zero = 0.0, thickness = 0.0;
    if (!(file >> zero >> thickness)) return c;
    c.containerZero_mm = zero;
    c.floaterThickness_mm = thickness;
    int off = 0;
    if (file >> off) {
        c.hasOffsetRecord = true;
        c.offsetMm = off;
    }
    return c;
}

void warnIfCalibrationStale(const Calibration& c, int currentOffsetMm) {
    if (!c.valid()) {
        std::cout << "[!] No calibration found. Run Hardware > Calibration first.\n";
    } else if (!c.hasOffsetRecord) {
        std::cout << "[!] " << cfg::kCalibrationFile << " has no offset record. If it was taken with a\n"
                  << "    different sensor offset (e.g. 0 mm), levels will be off. Re-run calibration.\n";
    } else if (c.offsetMm != currentOffsetMm) {
        std::cout << "[!] Calibration was taken with offset " << c.offsetMm << " mm but the sensor now uses "
                  << currentOffsetMm << " mm. Re-run calibration.\n";
    }
}

// Single source of truth for the fluid-level formula.
double computeFluidLevel(double zeroReference_mm, double rawDistance_mm, double floaterThickness_mm) {
    const double level = (zeroReference_mm - rawDistance_mm - floaterThickness_mm) * cfg::kLevelGainCorrection;
    return std::max(0.0, level);
}

// ============================================================================
// Application context
// ============================================================================
struct Context {
    ToFSensor tof;
    Calibration cal;
    std::string sessionId;
};

bool requireCalibration(const Context& ctx) {
    if (!ctx.cal.valid()) {
        std::cout << "[!] Run calibration first.\n";
        return false;
    }
    return true;
}

// ============================================================================
// Level monitoring (shared by every fill / drain / stream loop)
// ============================================================================
// Logs every level reading taken while pumping / draining (one row per reading).
class LiveLog {
public:
    LiveLog(const std::string& sessionId, const std::string& event)
        : csv_(cfg::kLiveDataFile,
               "SessionID,Timestamp,Event,Target_mm,RawMedian_mm,Level_mm,ValidSamples"),
          sessionId_(sessionId), event_(event) {}

    void record(double target_mm, const SensorMetrics& m, double level_mm) {
        if (!csv_.ok()) return;
        csv_.stream() << sessionId_ << ',' << timestamp("%Y-%m-%d %H:%M:%S") << ',' << event_ << ','
                      << fmt(target_mm, 1) << ',' << fmt(m.median, 1) << ',' << fmt(level_mm, 1) << ','
                      << m.validSamples << '\n';
    }

private:
    CsvFile csv_;
    std::string sessionId_;
    std::string event_;
};

enum class Outcome { Completed, KeyStop, Quit, Emergency, SensorFault };

void reportOutcome(Outcome o) {
    switch (o) {
        case Outcome::KeyStop:     std::cout << "\n[SYSTEM] Stopped by key press.\n"; break;
        case Outcome::Quit:        std::cout << "\n[SYSTEM] Quit requested.\n"; break;
        case Outcome::Emergency:   std::cout << "\n[SYSTEM] Emergency stop.\n"; break;
        case Outcome::SensorFault: std::cout << "\n[!] ToF returned no valid readings - stopped for safety.\n"; break;
        case Outcome::Completed:   break;
    }
}

// Reads the level repeatedly until step() returns true, a key is pressed ('q' => Quit),
// a signal arrives, or the sensor stops producing valid data.
template <typename Step>
Outcome monitorLevel(ToFSensor& tof, const Calibration& cal, int frames, Step step) {
    KeyPoller keys;
    keys.drain();
    int faults = 0;
    while (!g_emergencyStop) {
        if (keys.pressed()) {
            const int c = keys.readKey();
            return (c == 'q' || c == 'Q') ? Outcome::Quit : Outcome::KeyStop;
        }
        const SensorMetrics m = tof.measure(frames);
        if (m.validSamples == 0) {
            if (++faults >= cfg::kMaxFaultCycles) return Outcome::SensorFault;
            continue;
        }
        faults = 0;
        const double level = computeFluidLevel(cal.containerZero_mm, m.median, cal.floaterThickness_mm);
        if (step(level, m)) return Outcome::Completed;
    }
    return Outcome::Emergency;
}

bool readLevel(ToFSensor& tof, const Calibration& cal, int frames, double& level) {
    const SensorMetrics m = tof.measure(frames);
    if (m.validSamples == 0) return false;
    level = computeFluidLevel(cal.containerZero_mm, m.median, cal.floaterThickness_mm);
    return true;
}

// Runs the pump until level >= target. Pump is guaranteed off on return.
Outcome fillToLevel(ToFSensor& tof, const Calibration& cal, int target, int pumpPwm,
                    LiveLog* log = nullptr) {
    ScopedActuator pump(kPump, pumpPwm);
    return monitorLevel(tof, cal, 3, [&](double level, const SensorMetrics& m) {
        if (log) log->record(target, m, level);
        std::cout << "\rLvl: " << fmt(level, 1) << "/" << target << " mm | Yield: "
                  << m.validSamples << "/" << m.requestedSamples << "    " << std::flush;
        return level >= target;
    });
}

// Opens the solenoid until level <= stopLevel (never below the "empty" threshold).
Outcome drainToLevel(ToFSensor& tof, const Calibration& cal, double stopLevel, int solenoidPwm,
                     int frames, LiveLog* log = nullptr) {
    const double effectiveStop = std::max(stopLevel, cfg::kEmptyLevelMm);
    ScopedActuator valve(kSolenoid, solenoidPwm);
    return monitorLevel(tof, cal, frames, [&](double level, const SensorMetrics& m) {
        if (log) log->record(effectiveStop, m, level);
        std::cout << "\rLvl: " << fmt(level, 1) << " mm | Stop at: " << fmt(effectiveStop, 1)
                  << " mm | Yield: " << m.validSamples << "/" << m.requestedSamples << "    " << std::flush;
        return level <= effectiveStop;
    });
}

// Waits up to `ms`; returns false if the user pressed 'q' or an emergency stop occurred.
bool settleWait(int ms) {
    KeyPoller keys;
    keys.drain();
    for (int waited = 0; waited < ms && !g_emergencyStop; waited += 100) {
        if (keys.pressed()) {
            const int c = keys.readKey();
            if (c == 'q' || c == 'Q') return false;
        }
        sleepMs(100);
    }
    return !g_emergencyStop;
}

// ============================================================================
// Data logging
// ============================================================================
void captureRawReadings(Context& ctx, const std::string& eventName, const Calibration& cal) {
    std::cout << "\n[DATA LOG] Capturing " << cfg::kRawCaptureCount << " raw ToF readings (~20 seconds)...\n";
    CsvFile csv(cfg::kRawDataFile,
                "SessionID,Timestamp,Event,SampleIndex,RawDistance_mm,CalculatedLevel_mm");

    for (int i = 1; i <= cfg::kRawCaptureCount && !g_emergencyStop; ++i) {
        uint16_t d = 0;
        uint8_t status = 0;
        if (!ctx.tof.readFrame(d, status)) {
            std::cout << "\n[!] Sensor timeout during capture.\n";
            break;
        }
        const int raw = (status == 0) ? d : 0;
        double level = 0.0;
        if (cal.valid() && raw > 0) {
            level = computeFluidLevel(cal.containerZero_mm, raw, cal.floaterThickness_mm);
        }
        std::cout << "\rSample " << i << "/" << cfg::kRawCaptureCount << ": Raw " << raw
                  << " mm | Lvl " << fmt(level, 1) << " mm    " << std::flush;
        if (csv.ok()) {
            csv.stream() << ctx.sessionId << ',' << timestamp("%Y-%m-%d %H:%M:%S") << ',' << eventName << ','
                         << i << ',' << raw << ',' << fmt(level) << '\n';
        }
    }
    std::cout << "\n[DATA LOG] Capture complete.\n";
}

void logCycleData(const std::string& sessionId, int targetLevel, double calculatedLevel,
                  double actualMeasured, double rawMedian) {
    CsvFile csv(cfg::kDataFile,
                "SessionID,Timestamp,TargetLevel_mm,ToFCalculated_mm,ActualMeasured_mm,ToFRawAvg_mm,Error_mm");
    if (!csv.ok()) return;
    csv.stream() << sessionId << ',' << timestamp("%Y-%m-%d %H:%M:%S") << ',' << targetLevel << ','
                 << fmt(calculatedLevel) << ',' << fmt(actualMeasured) << ',' << fmt(rawMedian, 1) << ','
                 << fmt(actualMeasured - calculatedLevel) << '\n';
}

// ============================================================================
// Hardware control & setup
// ============================================================================
void runCalibration(Context& ctx) {
    std::cout << "\n--- [ CALIBRATION: SET SYSTEM ZERO ] ---\n";
    std::cout << "[!] Ensure the tank is completely DRAINED.\n";
    std::cout << "[!] Place the FLOATER inside (let it rest naturally on the gasket).\n";
    if (!confirm("Press ENTER to set system zero (or 'q' to cancel)... ")) return;

    captureRawReadings(ctx, "Calibration_SystemZero", Calibration());

    const SensorMetrics zero = ctx.tof.measure(20, true);
    if (zero.validSamples == 0) {
        std::cout << "[!] Calibration failed. Check sensor reading.\n";
        return;
    }

    Calibration c;
    c.containerZero_mm = zero.median;
    c.floaterThickness_mm = 0.0;
    c.hasOffsetRecord = true;
    c.offsetMm = ctx.tof.offsetMm();
    ctx.cal = c;

    std::cout << ">> System Zero (distance to resting floater): " << fmt(c.containerZero_mm)
              << " mm  (sensor offset " << c.offsetMm << " mm)\n";
    if (!saveCalibration(c)) std::cout << "[!] Could not save " << cfg::kCalibrationFile << ".\n";
}

void runContinuousRead(Context& ctx) {
    if (!requireCalibration(ctx)) return;
    std::cout << "\n--- [ CONTINUOUS SENSOR STREAM ] ---\nPress ANY KEY to stop.\n\n";
    const Outcome o = monitorLevel(ctx.tof, ctx.cal, 1, [](double level, const SensorMetrics& m) {
        std::cout << "\rLvl: " << fmt(level, 1) << " mm | Raw ToF: " << fmt(m.median, 0) << " mm    "
                  << std::flush;
        return false;
    });
    if (o != Outcome::KeyStop && o != Outcome::Quit) reportOutcome(o);
    std::cout << "\n";
}

void runSolenoidTestOnly() {
    std::cout << "\n--- [ SOLENOID TOGGLE (DRY TEST) ] ---\n";
    const int pwm = promptPwm("Solenoid");
    std::cout << "Press ENTER to OPEN valve, ENTER again to CLOSE. Type 'q' and ENTER to quit.\n";

    bool open = false;
    std::string line;
    while (!g_emergencyStop && readLine(line) && !isQuit(line)) {
        open = !open;
        setSolenoid(open, pwm);
        std::cout << "[SYSTEM] Solenoid is now " << (open ? "OPEN" : "CLOSED") << ".\n";
    }
    setSolenoid(false);
}

void runManualHardwareControl() {
    std::cout << "\n--- [ MANUAL HARDWARE CONTROL ] ---\n"
              << "Select Hardware:\n [1] Water Pump\n [2] Solenoid Valve\n";
    int hw = 0;
    if (!promptNumber("Selection: ", hw, 1, 2)) return;
    const Actuator& actuator = (hw == 1) ? kPump : kSolenoid;
    const int pwm = promptPwm(actuator.name);

    std::cout << "\nSelect Control Mode:\n [1] Toggle (ENTER to start, ENTER to stop)\n"
              << " [2] Timer (run for X seconds)\n";
    int mode = 0;
    if (!promptNumber("Selection: ", mode, 1, 2)) return;

    if (mode == 1) {
        if (!confirm(std::string("\nPress ENTER to turn ON the ") + actuator.name + "... ")) return;
        {
            ScopedActuator run(actuator, pwm);
            std::cout << "[" << actuator.name << " ON] Press ENTER to turn OFF...\n";
            std::string line;
            readLine(line);
        }
        std::cout << "[" << actuator.name << " OFF]\n";
        return;
    }

    int seconds = 0;
    if (!promptNumber("\nEnter duration in seconds: ", seconds, 1, 3600)) return;
    {
        ScopedActuator run(actuator, pwm);
        std::cout << "[" << actuator.name << " ON] Running for " << seconds
                  << " seconds. Press ANY KEY to abort.\n";
        KeyPoller keys;
        keys.drain();
        const auto start = std::chrono::steady_clock::now();
        while (!g_emergencyStop) {
            const long elapsed = static_cast<long>(
                std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now() - start).count());
            if (elapsed >= seconds) break;
            if (keys.pressed()) {
                std::cout << "\n[ABORTED] Manual interruption.\n";
                break;
            }
            std::cout << "\rRemaining: " << (seconds - elapsed) << " s   " << std::flush;
            sleepMs(100);
        }
    }
    std::cout << "\n[" << actuator.name << " OFF]\n";
}

// ============================================================================
// Fill test: tank-zero experiment
// ============================================================================
struct ExperimentParams {
    int target = 0;
    int iterations = 0;
    int physicalMeasures = 0;
    int pumpPwm = cfg::kPwmRange;
    int solenoidPwm = cfg::kPwmRange;
};

bool promptTankBottom(ToFSensor& tof, double& tankBottom) {
    std::cout << "\nHow would you like to set the Tank Bottom Zero?\n"
              << " [1] Auto-read via ToF sensor (needs an opaque target on clear tank floors)\n"
              << " [2] Manually enter known physical distance (recommended for clear acrylic)\n";
    int choice = 0;
    if (!promptNumber("Selection: ", choice, 1, 2)) return false;

    if (choice == 1) {
        std::cout << "\n[!] Ensure the tank is EMPTY and the FLOATER IS REMOVED.\n";
        if (!confirm("Press ENTER to read the Tank Bottom Distance (or 'q' to cancel)... ")) return false;
        const SensorMetrics m = tof.measure(20, true);
        if (m.validSamples == 0) {
            std::cout << "[!] Calibration failed. Check sensor reading.\n";
            return false;
        }
        tankBottom = m.median;
        std::cout << ">> Tank Bottom Zero (auto): " << fmt(tankBottom) << " mm\n\n";
    } else {
        if (!promptNumber("\nEnter physical distance from the sensor chip to the tank bottom (mm): ",
                          tankBottom, 1.0, 10000.0)) {
            return false;
        }
        std::cout << ">> Tank Bottom Zero (manual): " << fmt(tankBottom) << " mm\n\n";
    }
    return true;
}

// Returns false if the experiment should stop.
bool askPhysicalMeasurements(const Context& ctx, const ExperimentParams& p, int iter,
                             double settledAvg, double distFromZero, double calcLevel, CsvFile& csv) {
    std::cout << "[!] REMOVE the floater from the tank.\n";
    for (int i = 1; i <= p.physicalMeasures; ++i) {
        double value = 0.0;
        while (true) {
            std::cout << "Enter physical measurement #" << i << " (mm) [-1 to EXIT]: " << std::flush;
            std::string line;
            if (!readLine(line)) return false;
            if (parseNumber(line, value)) break;
            std::cout << "[!] Please enter a number.\n";
        }
        if (value < 0.0) return false;
        if (csv.ok()) {
            csv.stream() << ctx.sessionId << ',' << iter << ',' << p.target << ',' << fmt(settledAvg) << ','
                         << fmt(distFromZero) << ',' << fmt(calcLevel) << ',' << i << ',' << fmt(value) << '\n';
        }
    }
    std::cout << "[SYSTEM] Measurements saved.\n\n";
    return true;
}

// One fill -> settle -> measure -> drain pass. Returns false if the experiment should stop.
bool runExperimentIteration(Context& ctx, const Calibration& tank, const ExperimentParams& p,
                            int iter, CsvFile& csv) {
    std::cout << "\n=======================================\n"
              << " ITERATION " << iter << " / " << p.iterations
              << "\n=======================================\n";

    std::cout << "[!] Ensure FLOATER is IN the tank.\n";
    if (!confirm("Press ENTER to START PUMP and fill to " + std::to_string(p.target) +
                 " mm (or type 'q' to abort)... ")) {
        return false;
    }

    std::cout << "Filling... Press ANY KEY to stop pump early, or 'q' to EXIT experiment.\n";
    LiveLog fillLog(ctx.sessionId, "TankZero_Fill");
    Outcome o = fillToLevel(ctx.tof, tank, p.target, p.pumpPwm, &fillLog);
    if (o != Outcome::Completed && o != Outcome::KeyStop) {
        reportOutcome(o);
        return false;
    }

    std::cout << "\n[SYSTEM] Pump stopped. Waiting 2 seconds for fluid to settle (press 'q' to exit)...\n";
    if (!settleWait(2000)) return false;

    captureRawReadings(ctx, "TankZero_SettledMeasurement", tank);

    const SensorMetrics post = ctx.tof.measure(10, true);
    const double settledAvg = (post.validSamples > 0) ? post.median : tank.containerZero_mm;
    const double distFromZero = tank.containerZero_mm - settledAvg;
    const double calcLevel = computeFluidLevel(tank.containerZero_mm, settledAvg, tank.floaterThickness_mm);

    std::cout << "\n>> ToF median: " << fmt(settledAvg) << " mm\n"
              << ">> Calculated level: " << fmt(calcLevel) << " mm\n\n";

    if (!askPhysicalMeasurements(ctx, p, iter, settledAvg, distFromZero, calcLevel, csv)) return false;

    std::cout << "[!] PLACE FLOATER BACK IN THE TANK.\n";
    if (!confirm("Press ENTER to OPEN SOLENOID and start draining (or type 'q' to EXIT)... ")) return false;

    std::cout << "Draining... Press ANY KEY to stop solenoid, or 'q' to EXIT experiment.\n";
    LiveLog drainLog(ctx.sessionId, "TankZero_Drain");
    o = drainToLevel(ctx.tof, tank, 0.0, p.solenoidPwm, 2, &drainLog);
    if (o == Outcome::Completed) {
        std::cout << "\n[SYSTEM] Tank empty. Solenoid closed.\n";
    } else if (o != Outcome::KeyStop) {
        reportOutcome(o);
        return false;
    }
    std::cout << "\n[SYSTEM] Draining complete for iteration " << iter << ".\n";
    return true;
}

void runTankZeroExperiment(Context& ctx) {
    std::cout << "\n--- [ TANK ZERO DISTANCE EXPERIMENT (FILL & MONITOR) ] ---\n";

    Calibration tank;
    tank.hasOffsetRecord = true;
    tank.offsetMm = ctx.tof.offsetMm();
    if (!promptTankBottom(ctx.tof, tank.containerZero_mm)) return;

    std::cout << "[!] Place the FLOATER in the tank.\n";
    if (!confirm("Press ENTER to read floater resting distance (or 'q' to cancel)... ")) return;

    const SensorMetrics floater = ctx.tof.measure(20, true);
    if (floater.validSamples == 0) {
        std::cout << "[!] Floater resting-distance read failed (no valid ToF samples). Aborting.\n";
        return;
    }
    tank.floaterThickness_mm = tank.containerZero_mm - floater.median;
    std::cout << ">> Floater resting distance: " << fmt(floater.median, 1) << " mm\n"
              << ">> Calculated floater thickness: " << fmt(tank.floaterThickness_mm) << " mm\n";
    if (tank.floaterThickness_mm < 0.0) {
        std::cout << "[!] Negative thickness: the floater reads FARTHER than the tank bottom. "
                  << "Check the zero distance and sensor offset.\n";
    }
    std::cout << "\n";

    ExperimentParams p;
    if (!promptNumber("Enter target fluid level (mm): ", p.target, 1, 10000)) return;
    if (!promptNumber("Enter number of iterations for this level: ", p.iterations, 1, 1000)) return;
    if (!promptNumber("Enter number of physical measurements per iteration: ", p.physicalMeasures, 1, 100)) return;
    p.pumpPwm = promptPwm("Pump");
    p.solenoidPwm = promptPwm("Solenoid");

    CsvFile csv(cfg::kExperimentFile,
                "SessionID,Iteration,TargetLevel_mm,ToFAvg_mm,DistFromZero_mm,CalculatedFluid_mm,"
                "PhysicalIndex,PhysicalMeasure_mm");

    bool completed = true;
    for (int iter = 1; iter <= p.iterations && !g_emergencyStop; ++iter) {
        if (!runExperimentIteration(ctx, tank, p, iter, csv)) {
            completed = false;
            break;
        }
    }
    parkHardware();
    std::cout << (completed ? "\n[SYSTEM] Experiment sequence complete. Hardware parked.\n"
                            : "\n[SYSTEM] Experiment terminated early. Hardware parked.\n");
}

// ============================================================================
// Drain test
// ============================================================================
void runSolenoidAndToF(Context& ctx) {
    if (!requireCalibration(ctx)) return;
    std::cout << "\n--- [ GRAVITY DRAIN & MONITOR ] ---\n";
    const int pwm = promptPwm("Solenoid");
    if (!confirm("Press ENTER to OPEN SOLENOID and monitor ToF drop (or type 'q' to abort)... ")) return;

    std::cout << "Draining... Press ANY KEY to abort.\n";
    LiveLog log(ctx.sessionId, "GravityDrain");
    const Outcome o = drainToLevel(ctx.tof, ctx.cal, 0.0, pwm, 3, &log);
    if (o == Outcome::Completed) {
        std::cout << "\n[SYSTEM] Tank empty. Solenoid closed.\n";
    } else {
        reportOutcome(o);
    }
}

// ============================================================================
// Fill + drain test
// ============================================================================
void runFullFluidCycle(Context& ctx) {
    if (!requireCalibration(ctx)) return;
    std::cout << "\n--- [ FULL CYCLE: FILL -> SETTLE -> STEPPED DRAIN ] ---\n";

    const int pumpPwm = promptPwm("Pump");
    const int solenoidPwm = promptPwm("Solenoid");
    int target = 0, drainStep = 0;
    if (!promptNumber("Enter target fill level (mm): ", target, 1, 10000)) return;
    if (!promptNumber("Enter drain step interval (mm): ", drainStep, 1, 10000)) return;

    std::cout << "\n[PHASE 1] FILLING\n";
    if (!confirm("Press ENTER to START PUMP and fill to " + std::to_string(target) +
                 " mm (or type 'q' to abort)... ")) {
        return;
    }
    setSolenoid(false);
    std::cout << "Filling... Press ANY KEY to abort.\n";
    LiveLog fillLog(ctx.sessionId, "FullCycle_Fill");
    const Outcome fill = fillToLevel(ctx.tof, ctx.cal, target, pumpPwm, &fillLog);
    if (fill != Outcome::Completed) {
        reportOutcome(fill);
        parkHardware();
        return;
    }

    std::cout << "\n[SYSTEM] Target reached. Settling fluid...\n";
    if (!settleWait(1000)) return;

    captureRawReadings(ctx, "FullCycle_SettledMeasurement", ctx.cal);
    const SensorMetrics settled = ctx.tof.measure(5, true);
    if (settled.validSamples == 0) {
        std::cout << "\n[!] Settled-level read failed (no valid ToF samples) - aborting cycle.\n";
        parkHardware();
        return;
    }
    const double calculated =
        computeFluidLevel(ctx.cal.containerZero_mm, settled.median, ctx.cal.floaterThickness_mm);

    std::cout << "\n[PHASE 2] SETTLED (ToF calculated: " << fmt(calculated) << " mm)\n";
    double actual = 0.0;
    if (promptNumber("Enter actual measured fluid level (mm): ", actual)) {
        logCycleData(ctx.sessionId, target, calculated, actual, settled.median);
    }

    std::cout << "\n[PHASE 3] STEPPED DRAINING (step size: " << drainStep << " mm)\n";
    while (!g_emergencyStop) {
        double level = 0.0;
        if (!readLevel(ctx.tof, ctx.cal, 3, level)) {
            std::cout << "[!] ToF read failed - stopping drain sequence.\n";
            break;
        }
        if (level <= cfg::kEmptyLevelMm) break;

        std::cout << "Current level: " << fmt(level, 1) << " mm.\n";
        if (!confirm("Press ENTER to OPEN SOLENOID and drain " + std::to_string(drainStep) +
                     " mm (or 'q' to stop)... ")) {
            break;
        }

        std::cout << "Draining... Press ANY KEY to stop.\n";
        LiveLog drainLog(ctx.sessionId, "FullCycle_Drain");
        const Outcome o = drainToLevel(ctx.tof, ctx.cal, std::max(0.0, level - drainStep), solenoidPwm, 2, &drainLog);
        std::cout << "\n[STEP COMPLETE] Solenoid closed.\n";
        if (o == Outcome::Emergency || o == Outcome::SensorFault) {
            reportOutcome(o);
            break;
        }

        if (!settleWait(1000)) break;
        captureRawReadings(ctx, "FullCycle_PostDrainStep", ctx.cal);
        std::cout << "\n";
    }
    parkHardware();
    std::cout << "\n[SYSTEM] Cycle complete. Hardware parked.\n";
}

// ============================================================================
// Menus
// ============================================================================
struct MenuItem {
    std::string label;
    std::function<void()> action;
};

void runMenu(const std::string& title, const std::vector<MenuItem>& items, const std::string& backLabel) {
    const int count = static_cast<int>(items.size());
    while (!g_systemOffline) {
        std::cout << "\n--- " << title << " ---\n";
        for (int i = 0; i < count; ++i) {
            std::cout << " [" << (i + 1) << "] " << items[static_cast<size_t>(i)].label << "\n";
        }
        std::cout << " [" << (count + 1) << "] " << backLabel << "\n";

        int choice = 0;
        if (!promptNumber("Selection: ", choice, 1, count + 1)) continue;
        if (choice == count + 1) return;
        items[static_cast<size_t>(choice - 1)].action();
    }
}

void initHardware() {
    pinMode(cfg::kPumpEn, OUTPUT);
    pinMode(cfg::kPumpIna, OUTPUT);
    pinMode(cfg::kPumpInb, OUTPUT);
    pinMode(cfg::kPumpPwm, PWM_OUTPUT);

    pinMode(cfg::kSolenoidIna, OUTPUT);
    pinMode(cfg::kSolenoidInb, OUTPUT);
    pinMode(cfg::kSolenoidPwm, PWM_OUTPUT);

    parkHardware();
}

}  // namespace

// ============================================================================
// main
// ============================================================================
int main() {
    installSignalHandlers();

    if (wiringPiSetupGpio() == -1) {
        std::cerr << "Error: failed to initialize WiringPi.\n";
        return 1;
    }
    initHardware();

    Context ctx;
    ctx.sessionId = timestamp("%Y%m%d_%H%M%S");

    const int16_t offsetMm = loadSensorOffset();
    if (!ctx.tof.init(offsetMm) || !ctx.tof.startRanging()) {
        parkHardware();
        return 2;
    }

    ctx.cal = loadCalibration();
    warnIfCalibrationStale(ctx.cal, ctx.tof.offsetMm());

    const std::vector<MenuItem> hardwareMenu = {
        {"Set Container & Floater (Calibration)", [&] { runCalibration(ctx); }},
        {"Solenoid Toggle (Dry Test)", [] { runSolenoidTestOnly(); }},
        {"Manual Hardware Control (Toggle / Timer)", [] { runManualHardwareControl(); }},
        {"Continuous Sensor Stream", [&] { runContinuousRead(ctx); }},
    };
    const std::vector<MenuItem> fillMenu = {
        {"Tank Zero Distance Experiment (Fill & Monitor + Iterations)", [&] { runTankZeroExperiment(ctx); }},
    };
    const std::vector<MenuItem> drainMenu = {
        {"Gravity Drain & Monitor (Solenoid + ToF)", [&] { runSolenoidAndToF(ctx); }},
    };
    const std::vector<MenuItem> fillDrainMenu = {
        {"Full Cycle (Pump Fill -> Settle -> Stepped Drain)", [&] { runFullFluidCycle(ctx); }},
    };
    const std::vector<MenuItem> mainMenu = {
        {"Hardware Control & Setup", [&] { runMenu("HARDWARE CONTROL & SETUP", hardwareMenu, "Back to Main Menu"); }},
        {"Fill Tests", [&] { runMenu("FILL TESTS", fillMenu, "Back to Main Menu"); }},
        {"Drain Tests", [&] { runMenu("DRAIN TESTS", drainMenu, "Back to Main Menu"); }},
        {"Fill and Drain Tests", [&] { runMenu("FILL AND DRAIN TESTS", fillDrainMenu, "Back to Main Menu"); }},
    };

    runMenu("MIXR-1: FLUID DYNAMICS CONTROLLER", mainMenu, "Exit System");

    ctx.tof.stopRanging();
    parkHardware();
    std::cout << "\nSystem offline. Hardware safely parked.\n";
    return 0;
}
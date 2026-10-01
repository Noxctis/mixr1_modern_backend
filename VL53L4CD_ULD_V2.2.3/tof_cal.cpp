// tof_cal.cpp - VL53L4CD test + calibration tool (ST ULD driver, Raspberry Pi 4B)
//
// Build (adjust paths to match your project layout):
//   g++ -O2 -o tof_cal tof_cal.cpp \
//       VL53L4CD_ULD_Driver/VL53L4CD_api.c Platform/platform.c -I. -lm
//   (compile the two .c files with gcc if your toolchain complains:
//    gcc -c VL53L4CD_ULD_Driver/VL53L4CD_api.c Platform/platform.c)
//
// Usage:
//   ./tof_cal stats [n]            noise / validity check at the current distance
//   ./tof_cal offset <true_mm> [n] one-point offset calibration (saves tof_offset.txt)
//   ./tof_cal sweep                multi-point sweep -> linear fit (saves tof_fit.txt)
//   ./tof_cal verify               check corrected readings at new known distances
//
// Order: stats -> offset -> sweep -> verify.
// Re-run sweep any time you re-run offset (the fit is relative to the offset in use).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <map>
#include <sstream>
#include <string>
#include <vector>

extern "C" {
#include "VL53L4CD_ULD_Driver/VL53L4CD_api.h"
#include "Platform/platform.h"

// Declared explicitly in case an older platform.h is picked up first
uint8_t VL53L4CD_PlatformInit(void);
uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs);
}

static const Dev_t DEV = 0x52;                 // ULD uses the 8-bit form; platform.c ignores it
static const char* OFFSET_FILE = "tof_offset.txt";
static const char* FIT_FILE    = "tof_fit.txt";
static const char* POINTS_FILE = "tof_sweep_points.csv";

// ---------------------------------------------------------------- persistence
struct Fit { double m = 1.0, b = 0.0; };       // measured = m * true + b

static bool loadOffset(int16_t& off) {
    std::ifstream f(OFFSET_FILE);
    int v;
    if (f >> v) { off = (int16_t)v; return true; }
    return false;
}
static bool loadFit(Fit& fit) {
    std::ifstream f(FIT_FILE);
    return (bool)(f >> fit.m >> fit.b);
}
static double tofCorrect(double raw, const Fit& fit) { return (raw - fit.b) / fit.m; }

// ---------------------------------------------------------------- sensor setup
static bool tofInit(uint16_t budget_ms) {
    if (VL53L4CD_PlatformInit() != 0) return false;

    uint16_t id = 0;
    if (VL53L4CD_GetSensorId(DEV, &id) || id != 0xEBAA) {
        printf("VL53L4CD not detected (id=0x%04X)\n", id);
        return false;
    }
    if (VL53L4CD_SensorInit(DEV)) { printf("SensorInit failed\n"); return false; }
    if (VL53L4CD_SetRangeTiming(DEV, budget_ms, 0)) { printf("SetRangeTiming failed\n"); return false; }

    // Calibration is NOT stored in the sensor across power cycles: reload every boot.
    int16_t off = 0;
    if (loadOffset(off)) {
        VL53L4CD_SetOffset(DEV, off);
        printf("Loaded offset %d mm from %s\n", off, OFFSET_FILE);
    }
    // Optional: temperature compensation (sensor must not be ranging).
    // Re-run if ambient temperature changes by more than ~8 C.
    VL53L4CD_StartTemperatureUpdate(DEV);
    return true;
}

// ---------------------------------------------------------------- sampling
struct Samples {
    std::vector<double> d;
    std::map<int, int> status;   // range_status histogram
};

// Collects n VALID (range_status == 0) readings, discarding the first few frames.
static Samples collect(int n, int warmup = 5) {
    Samples s;
    VL53L4CD_StartRanging(DEV);
    int frames = 0, idle_ms = 0;
    const int maxFrames = warmup + 5 * n;
    while ((int)s.d.size() < n && frames < maxFrames && idle_ms < 2000) {
        uint8_t ready = 0;
        VL53L4CD_CheckForDataReady(DEV, &ready);
        if (!ready) { VL53L4CD_WaitMs(DEV, 2); idle_ms += 2; continue; }
        idle_ms = 0;
        VL53L4CD_ResultsData_t r;
        VL53L4CD_GetResult(DEV, &r);
        VL53L4CD_ClearInterrupt(DEV);
        if (++frames <= warmup) continue;
        s.status[r.range_status]++;
        if (r.range_status == 0) s.d.push_back(r.distance_mm);
    }
    VL53L4CD_StopRanging(DEV);
    return s;
}

static double median(std::vector<double> v) {
    std::sort(v.begin(), v.end());
    size_t n = v.size();
    return n % 2 ? v[n / 2] : 0.5 * (v[n / 2 - 1] + v[n / 2]);
}
static double mean(const std::vector<double>& v) {
    double s = 0; for (double x : v) s += x; return s / v.size();
}
static double stddev(const std::vector<double>& v) {
    double m = mean(v), s = 0; for (double x : v) s += (x - m) * (x - m);
    return v.size() > 1 ? std::sqrt(s / (v.size() - 1)) : 0.0;
}
static void printStatus(const Samples& s) {
    printf("  status histogram:");
    for (auto& kv : s.status) printf("  [%d]=%d", kv.first, kv.second);
    printf("   (0 = valid)\n");
}

// Returns false if there are too few valid samples to trust.
static bool measure(int n, double& med, double& sd, bool verbose = true) {
    Samples s = collect(n);
    if (verbose) printStatus(s);
    if ((int)s.d.size() < n / 2 || s.d.size() < 5) {
        printf("  Too few valid readings (%zu/%d). Check target, distance, ambient light.\n",
               s.d.size(), n);
        return false;
    }
    med = median(s.d);
    sd  = stddev(s.d);
    return true;
}

// ---------------------------------------------------------------- commands
static int cmdStats(int n) {
    Samples s = collect(n);
    printStatus(s);
    if (s.d.empty()) { printf("No valid readings.\n"); return 1; }
    printf("  valid=%zu  median=%.1f  mean=%.2f  stddev=%.2f  min=%.0f  max=%.0f mm\n",
           s.d.size(), median(s.d), mean(s.d), stddev(s.d),
           *std::min_element(s.d.begin(), s.d.end()),
           *std::max_element(s.d.begin(), s.d.end()));
    return 0;
}

static int cmdOffset(double trueMm, int n) {
    VL53L4CD_SetOffset(DEV, 0);                       // measure the raw sensor
    double med, sd;
    printf("Measuring raw distance at true = %.1f mm ...\n", trueMm);
    if (!measure(n, med, sd)) return 1;
    int16_t off = (int16_t)std::lround(trueMm - med);
    printf("  raw median = %.1f mm (sd %.2f)  ->  offset = %d mm\n", med, sd, off);

    VL53L4CD_SetOffset(DEV, off);
    std::ofstream(OFFSET_FILE) << off << "\n";

    double med2, sd2;
    printf("Verifying with offset applied ...\n");
    if (measure(n, med2, sd2)) printf("  median now = %.1f mm (target %.1f)\n", med2, trueMm);
    printf("Saved to %s (applied automatically on every start).\n", OFFSET_FILE);
    return 0;
}

static int cmdSweep(int n) {
    std::vector<double> xs, ys, sds;
    printf("Place a flat, matte target at each known distance (measure from the sensor's\n"
           "optical window, not the board edge). Blank line = done. Use 5+ points spanning\n"
           "your real working range, e.g. 30, 50, 75, 100, 150, 200, 250 mm.\n");
    while (true) {
        printf("true distance (mm): "); fflush(stdout);
        std::string line;
        if (!std::getline(std::cin, line) || line.empty()) break;
        double t = std::atof(line.c_str());
        double med, sd;
        if (!measure(n, med, sd)) continue;
        printf("  true=%.1f  measured=%.1f  sd=%.2f  error=%+.1f\n", t, med, sd, med - t);
        xs.push_back(t); ys.push_back(med); sds.push_back(sd);
    }
    if (xs.size() < 3) { printf("Need at least 3 points.\n"); return 1; }

    // Least squares: measured = m * true + b
    double xm = mean(xs), ym = mean(ys), sxx = 0, sxy = 0;
    for (size_t i = 0; i < xs.size(); i++) { sxx += (xs[i]-xm)*(xs[i]-xm); sxy += (xs[i]-xm)*(ys[i]-ym); }
    Fit fit; fit.m = sxy / sxx; fit.b = ym - fit.m * xm;

    printf("\nFit: measured = %.5f * true + %.3f\n", fit.m, fit.b);
    printf("%8s %10s %10s %12s %12s\n", "true", "measured", "sd", "err_before", "err_after");
    std::ofstream csv(POINTS_FILE);
    csv << "true_mm,measured_mm,sd_mm,err_before_mm,err_after_mm\n";
    double worstBefore = 0, worstAfter = 0;
    for (size_t i = 0; i < xs.size(); i++) {
        double eb = ys[i] - xs[i];
        double ea = tofCorrect(ys[i], fit) - xs[i];
        worstBefore = std::max(worstBefore, std::fabs(eb));
        worstAfter  = std::max(worstAfter,  std::fabs(ea));
        printf("%8.1f %10.1f %10.2f %+12.2f %+12.2f\n", xs[i], ys[i], sds[i], eb, ea);
        csv << xs[i] << "," << ys[i] << "," << sds[i] << "," << eb << "," << ea << "\n";
    }
    printf("Worst error: %.2f mm before fit, %.2f mm after fit\n", worstBefore, worstAfter);
    std::ofstream(FIT_FILE) << fit.m << " " << fit.b << "\n";
    printf("Saved %s and %s\n", FIT_FILE, POINTS_FILE);
    return 0;
}

static int cmdVerify(int n) {
    Fit fit;
    if (!loadFit(fit)) { printf("No %s - run sweep first.\n", FIT_FILE); return 1; }
    printf("Check at NEW distances you did not use in the sweep. Blank line = done.\n");
    while (true) {
        printf("true distance (mm): "); fflush(stdout);
        std::string line;
        if (!std::getline(std::cin, line) || line.empty()) break;
        double t = std::atof(line.c_str()), med, sd;
        if (!measure(n, med, sd, false)) continue;
        double c = tofCorrect(med, fit);
        printf("  raw=%.1f  corrected=%.1f  error=%+.2f mm  (sd %.2f)\n", med, c, c - t, sd);
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: %s stats [n] | offset <true_mm> [n] | sweep | verify\n", argv[0]);
        return 1;
    }
    // 200 ms budget = best accuracy (5 Hz). Use the same budget in your daemon
    // or the calibration will not match.
    if (!tofInit(200)) return 1;

    std::string cmd = argv[1];
    if (cmd == "stats")  return cmdStats(argc > 2 ? std::atoi(argv[2]) : 100);
    if (cmd == "offset" && argc > 2)
        return cmdOffset(std::atof(argv[2]), argc > 3 ? std::atoi(argv[3]) : 100);
    if (cmd == "sweep")  return cmdSweep(100);
    if (cmd == "verify") return cmdVerify(50);
    printf("unknown command\n");
    return 1;
}
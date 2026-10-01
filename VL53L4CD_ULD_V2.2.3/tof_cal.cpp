// tof_cal.cpp - VL53L4CD test + calibration tool (ST ULD driver, Raspberry Pi 4B)
//
// Build:  make        (see the Makefile next to this file)
//
// Usage:
//   ./tof_cal stats [n]            noise / validity check at the current distance
//   ./tof_cal offset <true_mm> [n] one-point offset calibration (saves tof_offset.txt)
//   ./tof_cal sweep                multi-point sweep -> linear fit (saves tof_fit.txt)
//   ./tof_cal verify               check corrected readings at new known distances
//
// Order: stats -> offset -> sweep -> verify.
// Re-run sweep any time you re-run offset (the fit is relative to the offset in use).
//
// RUN LABELS / LOGS
//   Every run gets a run id = start date-time (YYYYMMDD_HHMMSS). Every result is
//   APPENDED (never overwritten) to a CSV log with columns  run_id,timestamp,note,...
//     tof_stats_log.csv   tof_offset_log.csv   tof_sweep_log.csv
//     tof_fit_log.csv     tof_verify_log.csv
//   Add a free-text label to a run with the TOF_NOTE environment variable, e.g.
//     TOF_NOTE="whiteboard + paper" sudo -E ./tof_cal sweep
//   tof_offset.txt / tof_fit.txt always hold the LATEST calibration (what your daemon
//   loads); the run id that produced each is stored after the '#' in the file.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <string>
#include <vector>
#include <unistd.h>

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

static std::string RUN_ID, NOTE;
static int g_offset_mm = 0;                    // offset currently programmed in the sensor

// ---------------------------------------------------------------- run label + logging
static std::string stamp(const char* fmt) {
    std::time_t t = std::time(nullptr);
    char buf[40];
    std::strftime(buf, sizeof(buf), fmt, std::localtime(&t));
    return buf;
}

// Appends one row; writes the header first if the file is new/empty.
// `header` and `row` exclude the run_id,timestamp,note prefix columns.
static void logRow(const char* file, const char* header, const char* row) {
    bool fresh;
    { std::ifstream f(file); fresh = !(f.good() && f.peek() != std::ifstream::traits_type::eof()); }
    std::ofstream out(file, std::ios::app);
    if (fresh) out << "run_id,timestamp,note," << header << "\n";
    out << RUN_ID << "," << stamp("%Y-%m-%d %H:%M:%S") << "," << NOTE << "," << row << "\n";
}

// ---------------------------------------------------------------- persistence
struct Fit { double m = 1.0, b = 0.0; };       // measured = m * true + b

static bool loadOffset(int16_t& off, std::string& tag) {
    std::ifstream f(OFFSET_FILE);
    int v;
    if (f >> v) { off = (int16_t)v; std::getline(f, tag); return true; }
    return false;
}
static bool loadFit(Fit& fit, std::string& tag) {
    std::ifstream f(FIT_FILE);
    if (f >> fit.m >> fit.b) { std::getline(f, tag); return true; }
    return false;
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
    std::string tag;
    if (loadOffset(off, tag)) {
        VL53L4CD_SetOffset(DEV, off);
        g_offset_mm = off;
        printf("Loaded offset %d mm from %s %s\n", off, OFFSET_FILE, tag.c_str());
    } else {
        printf("WARNING: no %s in this folder -> running with NO offset correction\n", OFFSET_FILE);
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
    int rejected = 0;
    for (auto& kv : s.status) if (kv.first != 0) rejected += kv.second;
    double mn = *std::min_element(s.d.begin(), s.d.end());
    double mx = *std::max_element(s.d.begin(), s.d.end());
    printf("  valid=%zu  median=%.1f  mean=%.2f  stddev=%.2f  min=%.0f  max=%.0f mm\n",
           s.d.size(), median(s.d), mean(s.d), stddev(s.d), mn, mx);

    char row[200];
    snprintf(row, sizeof(row), "%d,%zu,%d,%.1f,%.2f,%.2f,%.0f,%.0f",
             g_offset_mm, s.d.size(), rejected, median(s.d), mean(s.d), stddev(s.d), mn, mx);
    logRow("tof_stats_log.csv",
           "offset_mm,valid,rejected,median_mm,mean_mm,stddev_mm,min_mm,max_mm", row);
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
    g_offset_mm = off;
    std::ofstream(OFFSET_FILE) << off << "  # run " << RUN_ID << "\n";

    double med2 = std::nan(""), sd2;
    printf("Verifying with offset applied ...\n");
    if (measure(n, med2, sd2)) printf("  median now = %.1f mm (target %.1f)\n", med2, trueMm);

    char row[160];
    snprintf(row, sizeof(row), "%.1f,%.1f,%.2f,%d,%.1f", trueMm, med, sd, (int)off, med2);
    logRow("tof_offset_log.csv", "true_mm,raw_median_mm,raw_sd_mm,offset_mm,verify_median_mm", row);
    printf("Saved to %s (applied automatically on every start). Logged to tof_offset_log.csv\n",
           OFFSET_FILE);
    return 0;
}

static int cmdSweep(int n) {
    std::vector<double> xs, ys, sds;
    printf("Place a flat, matte target at each known distance (measure from the sensor's\n"
           "optical window, not the board edge). Blank line = done. Use 5+ points spanning\n"
           "your real working range, e.g. 50, 75, 100, 125, 150, 175, 200, 225, 250 mm.\n"
           "Re-entering a distance REPLACES the earlier reading (retake).\n");
    while (true) {
        printf("true distance (mm): "); fflush(stdout);
        std::string line;
        if (!std::getline(std::cin, line) || line.empty()) break;
        double t = std::atof(line.c_str());
        double med, sd;
        if (!measure(n, med, sd)) continue;
        printf("  true=%.1f  measured=%.1f  sd=%.2f  error=%+.1f\n", t, med, sd, med - t);
        bool replaced = false;
        for (size_t i = 0; i < xs.size(); i++)
            if (xs[i] == t) { ys[i] = med; sds[i] = sd; replaced = true;
                              printf("  (replaced earlier reading at %.1f mm)\n", t); break; }
        if (!replaced) { xs.push_back(t); ys.push_back(med); sds.push_back(sd); }
    }
    if (xs.size() < 3) { printf("Need at least 3 points.\n"); return 1; }

    // Least squares: measured = m * true + b
    double xm = mean(xs), ym = mean(ys), sxx = 0, sxy = 0;
    for (size_t i = 0; i < xs.size(); i++) { sxx += (xs[i]-xm)*(xs[i]-xm); sxy += (xs[i]-xm)*(ys[i]-ym); }
    Fit fit; fit.m = sxy / sxx; fit.b = ym - fit.m * xm;

    printf("\nFit: measured = %.5f * true + %.3f\n", fit.m, fit.b);
    printf("%8s %10s %10s %12s %12s\n", "true", "measured", "sd", "err_before", "err_after");
    double worstBefore = 0, worstAfter = 0;
    for (size_t i = 0; i < xs.size(); i++) {
        double eb = ys[i] - xs[i];
        double ea = tofCorrect(ys[i], fit) - xs[i];
        worstBefore = std::max(worstBefore, std::fabs(eb));
        worstAfter  = std::max(worstAfter,  std::fabs(ea));
        printf("%8.1f %10.1f %10.2f %+12.2f %+12.2f%s\n", xs[i], ys[i], sds[i], eb, ea,
               std::fabs(ea) > 2.0 ? "   <-- check this point" : "");
        char row[160];
        snprintf(row, sizeof(row), "%d,%.1f,%.1f,%.2f,%.2f,%.2f",
                 g_offset_mm, xs[i], ys[i], sds[i], eb, ea);
        logRow("tof_sweep_log.csv", "offset_mm,true_mm,measured_mm,sd_mm,err_before_mm,err_after_mm", row);
    }
    printf("Worst error: %.2f mm before fit, %.2f mm after fit\n", worstBefore, worstAfter);
    if (worstAfter > 2.0 || std::fabs(fit.m - 1.0) > 0.02)
        printf("WARNING: poor fit (gain %.4f). Usually ONE bad point (wrong ruler distance, beam\n"
               "spilling off the target, glare). Re-measure the flagged point or leave it out and\n"
               "run sweep again before trusting tof_fit.txt.\n", fit.m);

    std::ofstream(FIT_FILE) << std::setprecision(10) << fit.m << " " << fit.b
                            << "  # run " << RUN_ID << "\n";
    char row[200];
    snprintf(row, sizeof(row), "%d,%.6f,%.4f,%zu,%.2f,%.2f",
             g_offset_mm, fit.m, fit.b, xs.size(), worstBefore, worstAfter);
    logRow("tof_fit_log.csv", "offset_mm,gain_m,intercept_b,n_points,worst_before_mm,worst_after_mm", row);
    printf("Saved %s (latest fit) and appended to tof_sweep_log.csv / tof_fit_log.csv\n", FIT_FILE);
    return 0;
}

static int cmdVerify(int n) {
    Fit fit; std::string tag;
    if (!loadFit(fit, tag)) { printf("No %s - run sweep first.\n", FIT_FILE); return 1; }
    printf("Using fit from %s %s\n", FIT_FILE, tag.c_str());
    printf("Check at NEW distances you did not use in the sweep. Blank line = done.\n");
    while (true) {
        printf("true distance (mm): "); fflush(stdout);
        std::string line;
        if (!std::getline(std::cin, line) || line.empty()) break;
        double t = std::atof(line.c_str()), med, sd;
        if (!measure(n, med, sd, false)) continue;
        double c = tofCorrect(med, fit);
        printf("  raw=%.1f  corrected=%.1f  error=%+.2f mm  (sd %.2f)\n", med, c, c - t, sd);
        char row[160];
        snprintf(row, sizeof(row), "%d,%.1f,%.1f,%.2f,%+.2f,%.2f",
                 g_offset_mm, t, med, c, c - t, sd);
        logRow("tof_verify_log.csv", "offset_mm,true_mm,raw_mm,corrected_mm,error_mm,sd_mm", row);
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc < 2) {
        printf("usage: %s stats [n] | offset <true_mm> [n] | sweep | verify\n"
               "label a run:  TOF_NOTE=\"whiteboard+paper\" sudo -E %s sweep\n", argv[0], argv[0]);
        return 1;
    }
    RUN_ID = stamp("%Y%m%d_%H%M%S");
    if (const char* e = std::getenv("TOF_NOTE")) {
        NOTE = e;
        for (char& c : NOTE) if (c == ',' || c == '\n' || c == '\r') c = ' ';
    }
    char cwd[512] = "?";
    if (!getcwd(cwd, sizeof(cwd))) cwd[0] = '?';
    printf("Run %s%s%s   (files in %s)\n", RUN_ID.c_str(),
           NOTE.empty() ? "" : "  note: ", NOTE.c_str(), cwd);

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
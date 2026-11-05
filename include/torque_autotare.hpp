// include/torque_autotare.hpp
#pragma once
#include <cmath>
#include <algorithm>
#include <cstdlib>
#include <string>
#include <vector>

// Automatic software tare for the torque sensor: re-zeroes it while the rig is idle, so slow amplifier drift
// (temperature, warm-up) does not end up in the measurements.
//
// A tare is only taken when ALL of these hold:
//   1. the motor is off (PWM 0 and no positive target),
//   2. the encoder has not moved more than `still_ticks` counts for `settle_s` seconds,
//   3. a full window of readings is quiet (std <= max_std) and not trending (half-to-half change <= max_drift),
//   4. the new zero lies within +-max_abs of the amplifier's own zero.
// (1) and (2) are both needed: a shaft can stand still with torque on it (motor stalled below breakaway PWM,
// fluid still swirling after a stop), and (3)/(4) catch the cases where something is still pushing on it.
//
// This corrects small drift on top of the amplifier's own calibration (SL/SH keys); it does not replace it.
// All thresholds are in the sensor's output unit (N*m with your current scale).
class TorqueAutoTare {
public:
    struct Settings {
        bool enabled = true;
        double settle_s = 3.0;      // stillness required before sampling starts (lets coasting and fluid swirl die out)
        long long still_ticks = 2;  // encoder jitter allowed at rest (same idea as Config::DEADBAND_TICK_THRESHOLD)
        size_t window = 40;         // readings averaged per tare
        double max_std = 0.005;     // reject a noisy window. Set to about 3x the idle std that --test-torque reports
        double max_drift = 0.005;   // reject a window whose second half differs from its first half by more than this
        double max_abs = 0.05;      // reject a zero this far from the amplifier zero: something is loading the shaft
    };

    explicit TorqueAutoTare(const Settings& s) : cfg_(s) { buf_.reserve(cfg_.window); }

    // Call every control loop. Returns true when a new tare was accepted (handy for a log line).
    //   now_s       any monotonic clock in seconds
    //   enc_count   encoder.get_sync_snapshot().count
    //   motor_off   PWM == 0 and target <= 0
    //   fresh/raw   a new RAW (untared) sensor reading this loop, if there was one
    bool update(double now_s, long long enc_count, bool motor_off, bool fresh, double raw) {
        if (!cfg_.enabled) return false;
        if (!motor_off) { anchored_ = false; buf_.clear(); return false; }
        if (!anchored_ || std::llabs(enc_count - anchor_) > cfg_.still_ticks) {
            anchored_ = true; anchor_ = enc_count; still_since_ = now_s; buf_.clear();
            return false;
        }
        if (now_s - still_since_ < cfg_.settle_s) return false;
        if (!fresh || !std::isfinite(raw)) return false;

        buf_.push_back(raw);
        if (buf_.size() < cfg_.window) return false;

        const size_t n = buf_.size(), h = n / 2;
        double mean = 0.0, a = 0.0, b = 0.0;
        for (size_t i = 0; i < n; ++i) { mean += buf_[i]; (i < h ? a : b) += buf_[i]; }
        mean /= n; a /= h; b /= (n - h);
        double var = 0.0;
        for (double v : buf_) var += (v - mean) * (v - mean);
        const double sd = std::sqrt(var / n);
        buf_.clear();                                        // next window starts fresh (keeps tracking while idle)

        if (sd > cfg_.max_std)                return reject("noisy", mean, sd);
        if (std::abs(b - a) > cfg_.max_drift) return reject("drifting", mean, sd);
        if (std::abs(mean) > cfg_.max_abs)    return reject("too far from zero (load on shaft?)", mean, sd);
        tare_ = mean; last_sd_ = sd; ++accepted_;
        return true;
    }

    double apply(double raw) const { return raw - tare_; }  // tared value; NaN stays NaN
    double tare() const { return tare_; }
    double last_std() const { return last_sd_; }
    int accepted() const { return accepted_; }
    int rejected() const { return rejected_; }
    const std::string& last_reject() const { return last_reject_; }
    const Settings& settings() const { return cfg_; }

private:
    Settings cfg_;
    std::vector<double> buf_;
    bool anchored_ = false;
    long long anchor_ = 0;
    double still_since_ = 0.0;
    double tare_ = 0.0, last_sd_ = 0.0;
    int accepted_ = 0, rejected_ = 0;
    std::string last_reject_;

    bool reject(const char* why, double mean, double sd) {
        ++rejected_;
        last_reject_ = std::string(why) + " (mean " + std::to_string(mean) + ", std " + std::to_string(sd) + ")";
        return false;
    }
};

// Shared command-line flags (test mode and live mode). Returns true if `arg` was one of them.
//   --no-autotare  --autotare-settle=S  --autotare-window=N  --autotare-max-std=X  --autotare-max-drift=X  --autotare-max-abs=X
inline bool parse_autotare_arg(const std::string& arg, TorqueAutoTare::Settings& s) {
    if (arg == "--no-autotare") { s.enabled = false; return true; }
    if (arg == "--autotare")    { s.enabled = true;  return true; }
    if (arg.rfind("--autotare-settle=", 0) == 0)    { s.settle_s  = std::stod(arg.substr(18)); return true; }
    if (arg.rfind("--autotare-window=", 0) == 0)    { s.window    = static_cast<size_t>(std::max(4, std::stoi(arg.substr(18)))); return true; }
    if (arg.rfind("--autotare-max-std=", 0) == 0)   { s.max_std   = std::stod(arg.substr(19)); return true; }
    if (arg.rfind("--autotare-max-drift=", 0) == 0) { s.max_drift = std::stod(arg.substr(21)); return true; }
    if (arg.rfind("--autotare-max-abs=", 0) == 0)   { s.max_abs   = std::stod(arg.substr(19)); return true; }
    return false;
}
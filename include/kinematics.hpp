// include/kinematics.hpp
#pragma once
#include "encoder.hpp"
#include <chrono>

struct KinematicsState {
    double exact_rpm;
    double ema_filtered_rpm;
    // Diagnostics of this call (used by the PPR study; the control loop ignores them)
    bool      new_snapshot = false;   // true when this call consumed a NEW aligned snapshot (the estimate was refreshed)
    long long delta_count  = 0;       // counts between the two snapshots used (0 when held)
    uint32_t  delta_tick_us = 0;      // elapsed time between them in us (0 when held)
    uint32_t  end_tick = 0;           // time stamp of the newest snapshot / edge that ended (or would end) the interval
};

class KinematicsEngine {
private:
    EncoderSnapshot prev_snapshot{0, 0};
    std::chrono::steady_clock::time_point last_pulse_time;
    double ema_filtered_rpm = 0.0;
    double last_calculated_rpm = 0.0;
    bool first_run = true;
    EncoderEdges prev_edges{};
    // State of estimator 3 ("dct"), the earlier discrete-time method
    std::chrono::steady_clock::time_point dct_last_time{};
    long long dct_last_count = 0;
    long long dct_ticks = 0;
    std::chrono::duration<double> dct_time{0.0};
    double dct_rpm = 0.0;
    bool dct_primed = false;
    bool edges_primed = false;
    KinematicsState finish(bool refreshed, long long dc, uint32_t dt, uint32_t end_tick);   // timeout, EMA, diagnostics (shared by both estimators)

public:
    void reset(EncoderSnapshot initial_snapshot);
    // Estimator 1 (default, "aligned"): interval ends at the last edge whose count is a multiple of 4 (age up to one encoder period T_a).
    KinematicsState process(EncoderSnapshot current_snapshot, int current_pwm, bool update_lcd);
    // Estimator 2 ("newest"), the synchronous CET of Anuchin, Dianov and Briz: the interval ends at the NEWEST edge of any type (age at most
    // one state, T_a/4) and starts at the most recent edge OF THE SAME TYPE as it was at the previous estimation. Both are whole periods.
    KinematicsState process_newest(const EncoderEdges& current, bool update_lcd);
    // Estimator 3 ("dct"), the earlier discrete-time method: count and clock are read every loop and accumulated until the elapsed
    // time reaches RPM_SAMPLE_WINDOW_US; then rpm = |ticks / CPR| * 60 / time, held until the next window completes.
    KinematicsState process_dct(long long current_count, int current_pwm);
};
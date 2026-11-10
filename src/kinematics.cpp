// src/kinematics.cpp
#include "kinematics.hpp"
#include "config.hpp"

void KinematicsEngine::reset(EncoderSnapshot initial_snapshot) {
    prev_snapshot = initial_snapshot;
    edges_primed = false;                       // the newest-edge estimator re-primes itself from the next call
    last_pulse_time = std::chrono::steady_clock::now();
    ema_filtered_rpm = 0.0;
    last_calculated_rpm = 0.0;
    first_run = true;
}

KinematicsState KinematicsEngine::finish(bool refreshed, long long dc, uint32_t dt, uint32_t end_tick) {
    const auto now = std::chrono::steady_clock::now();
    if (refreshed) {
        last_calculated_rpm = (static_cast<double>(dc) / Config::ENCODER_CPR) * (60000000.0 / static_cast<double>(dt));
        last_calculated_rpm *= Config::ENCODER_DIRECTION;
        last_pulse_time = now;
    } else {
        auto ms_since_pulse = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pulse_time).count();
        if (ms_since_pulse > 100) { // 100ms without a hardware pulse = 0 RPM
            last_calculated_rpm = 0.0;
        }
    }

    // Apply EMA filter
    if (first_run) {
        ema_filtered_rpm = last_calculated_rpm;
        first_run = false;
    } else {
        ema_filtered_rpm = (Config::RPM_ALPHA * last_calculated_rpm) + ((1.0 - Config::RPM_ALPHA) * ema_filtered_rpm);
    }

    KinematicsState out{last_calculated_rpm, ema_filtered_rpm};
    out.new_snapshot = refreshed;
    out.delta_count = refreshed ? dc : 0;
    out.delta_tick_us = refreshed ? dt : 0;
    out.end_tick = end_tick;
    return out;
}

KinematicsState KinematicsEngine::process(EncoderSnapshot current_snapshot, int /*current_pwm*/, bool /*update_lcd*/) {
    const uint32_t delta_tick = current_snapshot.tick - prev_snapshot.tick;
    const long long delta_count = wrapped_delta(current_snapshot.count, prev_snapshot.count);   // stays correct if the 32-bit count wraps
    // The snapshot passed here is guaranteed to be a multiple of 4, so delta_count is always a whole number of encoder periods.
    const bool refreshed = (delta_count != 0 && delta_tick != 0);
    if (refreshed) prev_snapshot = current_snapshot;
    return finish(refreshed, delta_count, delta_tick, current_snapshot.tick);
}

KinematicsState KinematicsEngine::process_newest(const EncoderEdges& cur, bool /*update_lcd*/) {
    int nw = 0;                                                     // newest edge = the register with the latest time stamp
    for (int i = 1; i < 4; ++i)
        if (static_cast<int32_t>(cur.e[i].tick - cur.e[nw].tick) > 0) nw = i;
    if (!edges_primed) {
        prev_edges = cur;
        edges_primed = true;
        return finish(false, 0, 0, cur.e[nw].tick);
    }
    const EncoderSnapshot& end = cur.e[nw];
    const EncoderSnapshot& start = prev_edges.e[nw];                // the same edge type, as it was at the previous estimation
    const uint32_t dt = end.tick - start.tick;
    const long long dc = wrapped_delta(end.count, start.count);
    if (dc != 0 && (dc & 3) != 0) {                                 // not a whole number of periods: an edge was lost (or a register was never filled)
        prev_edges = cur;                                           // re-synchronise on the next estimation
        return finish(false, 0, 0, end.tick);
    }
    const bool refreshed = (dc != 0 && dt != 0);
    if (refreshed) prev_edges = cur;                                // else: no new edge of this type yet, hold and let the interval grow
    return finish(refreshed, dc, dt, end.tick);
}
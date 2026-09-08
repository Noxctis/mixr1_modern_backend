// src/kinematics.cpp
#include "kinematics.hpp"
#include "config.hpp"

void KinematicsEngine::reset(EncoderSnapshot initial_snapshot) {
    prev_snapshot = initial_snapshot;
    last_pulse_time = std::chrono::steady_clock::now();
    ema_filtered_rpm = 0.0;
    last_calculated_rpm = 0.0;
    first_run = true;
}

KinematicsState KinematicsEngine::process(EncoderSnapshot current_snapshot, int current_pwm, bool update_lcd) {
    uint32_t delta_tick = current_snapshot.tick - prev_snapshot.tick;
    long long delta_count = current_snapshot.count - prev_snapshot.count;
    auto now = std::chrono::steady_clock::now();

    // The snapshot passed here is guaranteed to be a multiple of 4,
    // so delta_count will always be physically symmetric.
    if (delta_count != 0 && delta_tick != 0) {
        last_calculated_rpm = (static_cast<double>(delta_count) / Config::ENCODER_CPR) * (60000000.0 / static_cast<double>(delta_tick));
        last_calculated_rpm *= Config::ENCODER_DIRECTION; 
        
        prev_snapshot = current_snapshot; 
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

    return {last_calculated_rpm, ema_filtered_rpm};
}
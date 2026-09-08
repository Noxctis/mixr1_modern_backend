// src/kinematics.cpp
#include "kinematics.hpp"
#include "config.hpp"

void KinematicsEngine::reset(EncoderSnapshot initial_snapshot) {
    prev_snapshot = initial_snapshot;
    last_pulse_time = std::chrono::steady_clock::now();
    ema_filtered_rpm = 0.0;
    last_calculated_rpm = 0.0;
    first_run = true;
    hardware_synced = false; // Disarm the engine on reset
}

KinematicsState KinematicsEngine::process(EncoderSnapshot current_snapshot, int current_pwm, bool update_lcd) {
    uint32_t delta_tick = current_snapshot.tick - prev_snapshot.tick;
    long long delta_count = current_snapshot.count - prev_snapshot.count;
    auto now = std::chrono::steady_clock::now();

    if (delta_count != 0) {
        if (!hardware_synced) {
            // First pulse detected. Do not calculate RPM.
            // Consume the pulse purely to establish a live hardware timestamp.
            prev_snapshot = current_snapshot;
            last_pulse_time = now;
            hardware_synced = true;
            last_calculated_rpm = 0.0;
        } else if (delta_tick != 0) {
            // Reject massive delta_tick values caused by uptime initialization wrapping
            if (delta_tick > 100000) { 
                last_calculated_rpm = 0.0;
            } else {
                last_calculated_rpm = (static_cast<double>(delta_count) / Config::ENCODER_CPR) * (60000000.0 / static_cast<double>(delta_tick));
                last_calculated_rpm *= Config::ENCODER_DIRECTION; 
            }
            prev_snapshot = current_snapshot; 
            last_pulse_time = now;
        }
    } else {
        auto ms_since_pulse = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_pulse_time).count();
        if (ms_since_pulse > 100) { 
            last_calculated_rpm = 0.0;
            hardware_synced = false; // Motor stopped. Disarm the engine to protect against the next startup jolt.
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
// src/kinematics.cpp
#include "kinematics.hpp"
#include <cmath>

void KinematicsEngine::reset(long long current_encoder_count) {
    last_time = std::chrono::high_resolution_clock::now();
    last_count = current_encoder_count;
    sample_time = std::chrono::duration<double>::zero();
    sample_ticks = 0;
    last_sampled_rpm = 0.0;
    ema_rpm = 0.0;
    sma_sum = 0.0;
    sma_index = 0;
    sma_count = 0;
    sma_history.fill(0.0);
}

KinematicsEngine::TelemetryState KinematicsEngine::process(long long current_count, int current_pwm, bool update_sma) {
    TelemetryState state{0.0, 0.0, 0.0};

    auto current_time = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double> dt_sec = current_time - last_time;
    last_time = current_time;

    long long delta_ticks = current_count - last_count;
    last_count = current_count;

    delta_ticks *= Config::ENCODER_DIRECTION;
    sample_ticks += delta_ticks;
    sample_time += dt_sec;

    if (sample_time.count() >= static_cast<double>(Config::RPM_SAMPLE_WINDOW_US) / 1000000.0) {
        if (current_pwm == 0 && std::abs(sample_ticks) <= Config::DEADBAND_TICK_THRESHOLD) {
            last_sampled_rpm = 0.0; 
            ema_rpm = 0.0;
            sma_history.fill(0.0);
            sma_sum = 0.0;
        } else {
            last_sampled_rpm = std::abs((static_cast<double>(sample_ticks) / Config::ENCODER_CPR) * (60.0 / sample_time.count()));
        }
        sample_ticks = 0;
        sample_time = std::chrono::duration<double>::zero();
    }

    state.exact_rpm = last_sampled_rpm;
    ema_rpm = (Config::RPM_ALPHA * state.exact_rpm) + ((1.0 - Config::RPM_ALPHA) * ema_rpm);
    state.ema_filtered_rpm = ema_rpm;

    if (update_sma) {
        sma_sum -= sma_history[sma_index];
        sma_history[sma_index] = ema_rpm;
        sma_sum += sma_history[sma_index];
        
        sma_index = (sma_index + 1) % Config::SMA_WINDOW_SIZE;
        if (sma_count < Config::SMA_WINDOW_SIZE) sma_count++;
    }
    
    state.sma_ui_rpm = (sma_count == 0) ? 0.0 : (sma_sum / static_cast<double>(sma_count));
    return state;
}
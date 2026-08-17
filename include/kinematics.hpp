// include/kinematics.hpp
#pragma once
#include <chrono>
#include <array>
#include "config.hpp"

class KinematicsEngine {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    long long last_count = 0;
    std::chrono::duration<double> sample_time{0.0};
    long long sample_ticks = 0;
    double last_sampled_rpm = 0.0;
    
    double ema_rpm = 0.0;
    std::array<double, Config::SMA_WINDOW_SIZE> sma_history{};
    size_t sma_index = 0;
    double sma_sum = 0.0;
    size_t sma_count = 0;

public:
    struct TelemetryState {
        double exact_rpm;           
        double ema_filtered_rpm;    
        double sma_ui_rpm;          
    };

    void reset(long long current_encoder_count);
    [[nodiscard]] TelemetryState process(long long current_count, int current_pwm, bool update_sma);
};
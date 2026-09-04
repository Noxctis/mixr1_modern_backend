// include/kinematics.hpp
#pragma once
#include "encoder.hpp"
#include <chrono>

struct KinematicsState {
    double exact_rpm;
    double ema_filtered_rpm;
};

class KinematicsEngine {
private:
    EncoderSnapshot prev_snapshot{0, 0};
    std::chrono::steady_clock::time_point last_pulse_time;
    double ema_filtered_rpm = 0.0;
    double last_calculated_rpm = 0.0;
    bool first_run = true;

public:
    void reset(EncoderSnapshot initial_snapshot);
    KinematicsState process(EncoderSnapshot current_snapshot, int current_pwm, bool update_lcd);
};
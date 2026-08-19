// src/pi_controller.cpp
#include "pi_controller.hpp"
#include "config.hpp"

// Initialize with the lowest tier by default
PIController::PIController() : Kp(Config::PI_SCHEDULE[0].Kp), Ki(Config::PI_SCHEDULE[0].Ki) {}

void PIController::update_gains(double setpoint_rpm) {
    for (const auto& tier : Config::PI_SCHEDULE) {
        if (setpoint_rpm <= tier.rpm_threshold) {
            Kp = tier.Kp;
            Ki = tier.Ki;
            return;
        }
    }
    // Fallback: If RPM exceeds the table, use the highest speed tier safely
    Kp = Config::PI_SCHEDULE.back().Kp;
    Ki = Config::PI_SCHEDULE.back().Ki;
}

void PIController::reset() { 
    integral_sum = 0.0; 
}

int PIController::compute(double setpoint_rpm, double current_rpm, double dt) {
    if (dt <= 0.0 || setpoint_rpm <= 0.0) return 0;

    // 1. Dynamically swap gains based on target fluid speed
    update_gains(setpoint_rpm);

    // 2. Standard PI execution
    double error = setpoint_rpm - current_rpm;
    integral_sum += error * dt;

    if (Ki > 0) {
        if (integral_sum > (max_pwm / Ki)) integral_sum = (max_pwm / Ki);
        else if (integral_sum < -(max_pwm / Ki)) integral_sum = -(max_pwm / Ki);
    }

    double output = (Kp * error) + (Ki * integral_sum);

    if (output > max_pwm) output = max_pwm;
    else if (output < 0) output = 0; 
    
    return static_cast<int>(output);
}
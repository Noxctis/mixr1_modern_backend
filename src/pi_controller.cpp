// src/pi_controller.cpp
#include "pi_controller.hpp"
#include "config.hpp"

PIController::PIController() : Kp(Config::GLOBAL_KP), Ki(Config::GLOBAL_KI) {}

void PIController::reset() { 
    integral_sum = 0.0; 
}

int PIController::compute(double setpoint_rpm, double current_rpm, double dt) {
    if (dt <= 0.0 || setpoint_rpm <= 0.0) return 0;

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
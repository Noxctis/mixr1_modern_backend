#include "pi_controller.hpp"
#include "config.hpp"

PIController::PIController() : Kp(Config::PI_SCHEDULE[0].Kp), Ki(Config::PI_SCHEDULE[0].Ki) {}

void PIController::update_gains(double setpoint_rpm) {
    // Boundary 1: Below minimum mapping
    if (setpoint_rpm <= Config::PI_SCHEDULE.front().rpm) {
        Kp = Config::PI_SCHEDULE.front().Kp;
        Ki = Config::PI_SCHEDULE.front().Ki;
        return;
    }
    
    // Boundary 2: Above maximum mapping
    if (setpoint_rpm >= Config::PI_SCHEDULE.back().rpm) {
        Kp = Config::PI_SCHEDULE.back().Kp;
        Ki = Config::PI_SCHEDULE.back().Ki;
        return;
    }
    
    // Core: Linear Interpolation between mapped bisection points
    for (size_t i = 0; i < Config::PI_SCHEDULE.size() - 1; ++i) {
        const auto& lower = Config::PI_SCHEDULE[i];
        const auto& upper = Config::PI_SCHEDULE[i+1];

        if (setpoint_rpm >= lower.rpm && setpoint_rpm <= upper.rpm) {
            double range = upper.rpm - lower.rpm;
            double fraction = (setpoint_rpm - lower.rpm) / range;
            
            // y = y1 + fraction * (y2 - y1)
            Kp = lower.Kp + fraction * (upper.Kp - lower.Kp);
            Ki = lower.Ki + fraction * (upper.Ki - lower.Ki);
            return;
        }
    }
}

void PIController::reset() { 
    integral_sum = 0.0; 
}

int PIController::compute(double setpoint_rpm, double current_rpm, double dt) {
    if (dt <= 0.0 || setpoint_rpm <= 0.0) return 0;

    // Dynamically calculate exact gains for current fluid physics
    update_gains(setpoint_rpm);

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
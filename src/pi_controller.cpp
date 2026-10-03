// src/pi_controller.cpp
#include "pi_controller.hpp"
#include "config.hpp"
#include <algorithm>

void PIController::reset() {
    integral_sum = 0.0;
    primed = false;          // next compute() re-seeds the filter from the live reading
}

int PIController::compute(double setpoint_rpm, double current_rpm, double dt) {
    if (dt <= 0.0) return 0;

    // ---- Feedback low-pass (first-order):  y = a*x + (1-a)*y_prev ----
    // a = 1.0 -> no filtering. Lower a = flatter but laggier.
    const double a = std::clamp(Config::FEEDBACK_ALPHA, 0.01, 1.0);
    if (!primed) { filtered_rpm = current_rpm; primed = true; }
    else         { filtered_rpm = a * current_rpm + (1.0 - a) * filtered_rpm; }

    if (setpoint_rpm <= 0.0) { integral_sum = 0.0; return 0; }

    // Gains are read every call so live updates (CMD:KP / CMD:KI / --kp= / --ki=) apply immediately.
    const double Kp = Config::GLOBAL_KP;
    const double Ki = Config::GLOBAL_KI;

    const double error = setpoint_rpm - filtered_rpm;
    integral_sum += error * dt;

    if (Ki > 0.0) {
        const double lim = max_pwm / Ki;                    // anti-windup clamp
        integral_sum = std::clamp(integral_sum, -lim, lim);
    }

    double output = (Kp * error) + (Ki * integral_sum);
    output = std::clamp(output, 0.0, max_pwm);
    return static_cast<int>(output);
}
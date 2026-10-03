// include/pi_controller.hpp
#pragma once

class PIController {
private:
    double integral_sum = 0.0;
    double max_pwm = 4095.0;
    double filtered_rpm = 0.0;   // feedback low-pass state
    bool   primed = false;       // first sample seeds the filter (no startup jump)

public:
    PIController() = default;
    void reset();
    int compute(double setpoint_rpm, double current_rpm, double dt);

    // The RPM value the controller is actually acting on (after the feedback filter).
    // Returns `fallback` until the filter has been seeded.
    double feedback_rpm(double fallback) const { return primed ? filtered_rpm : fallback; }
};
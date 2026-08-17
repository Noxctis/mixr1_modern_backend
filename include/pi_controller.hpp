// include/pi_controller.hpp
#pragma once

class PIController {
private:
    double Kp, Ki;
    double integral_sum = 0.0;
    double max_pwm = 4095.0; 

public:
    PIController(double kp, double ki);
    void reset();
    int compute(double setpoint_rpm, double current_rpm, double dt);
};
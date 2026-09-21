// include/motor.hpp
#pragma once

class MotorController {
private:
    int pi_handle;
    int adc_handle; // NEW: Handle for ADS1115 I2C

public:
    explicit MotorController(int pi);
    ~MotorController();
    void set_pwm(int duty_cycle);
    void stop_motor();

    // NEW: Current and Power sensing
    double get_current_amps();
    double get_power_watts(int current_pwm);
};
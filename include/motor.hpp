// include/motor.hpp
#pragma once
#include <chrono>

class MotorController {
private:
    int pi_handle;
    int adc_handle; 
    
    // Cache variables to prevent I2C blocking
    std::chrono::steady_clock::time_point last_adc_time;
    double cached_current;

public:
    explicit MotorController(int pi);
    ~MotorController();
    void set_pwm(int duty_cycle);
    void stop_motor();

    double get_current_amps();
    double get_power_watts(int current_pwm);
};
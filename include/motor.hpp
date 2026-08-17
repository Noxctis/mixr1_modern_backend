// include/motor.hpp
#pragma once

class MotorController {
private:
    int pi_handle;

public:
    explicit MotorController(int pi);
    ~MotorController();
    void set_pwm(int duty_cycle);
    void stop_motor();
};
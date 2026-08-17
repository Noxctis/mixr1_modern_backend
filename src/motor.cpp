// src/motor.cpp
#include "motor.hpp"
#include "config.hpp"
#include <pigpiod_if2.h>
#include <iostream>
#include <algorithm>

MotorController::MotorController(int pi) : pi_handle(pi) {
    set_mode(pi_handle, Config::PIN_M1_EN, PI_OUTPUT);
    set_mode(pi_handle, Config::PIN_M1_INA, PI_OUTPUT);
    set_mode(pi_handle, Config::PIN_M1_INB, PI_OUTPUT);
    set_mode(pi_handle, Config::PIN_M1_PWM, PI_ALT0);

    gpio_write(pi_handle, Config::PIN_M1_EN, 1);
    gpio_write(pi_handle, Config::PIN_M1_INA, 1);
    gpio_write(pi_handle, Config::PIN_M1_INB, 0); 
    
    if (hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, 0) != 0) {
        std::cerr << "[CRITICAL] Silicon PWM rejected on GPIO 13\n";
    }
}

MotorController::~MotorController() { 
    stop_motor(); 
}

void MotorController::set_pwm(int duty_cycle) {
    duty_cycle = std::clamp(duty_cycle, 0, 4095);
    long long hw_duty = (static_cast<long long>(duty_cycle) * 1000000LL) / 4095LL;
    hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, static_cast<int>(hw_duty));
}

void MotorController::stop_motor() {
    hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, 0);
    gpio_write(pi_handle, Config::PIN_M1_EN, 0);
}
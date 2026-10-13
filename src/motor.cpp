// src/motor.cpp
#include "motor.hpp"
#include "config.hpp"
#include <pigpiod_if2.h>
#include <iostream>
#include <algorithm>

MotorController::MotorController(int pi) : pi_handle(pi) {
    // SAFE START ORDER: the driver stays DISABLED (EN low) until the PWM output has been forced to 0 %.
    // pigpiod does NOT reset the hardware PWM when a program is killed or crashes, so a stale duty from an earlier run
    // would otherwise be applied the instant EN goes high (a burst of full power at start-up).
    set_mode(pi_handle, Config::PIN_M1_EN, PI_OUTPUT);
    gpio_write(pi_handle, Config::PIN_M1_EN, 0);                 // 1) driver OFF first
    set_mode(pi_handle, Config::PIN_M1_INA, PI_OUTPUT);
    set_mode(pi_handle, Config::PIN_M1_INB, PI_OUTPUT);
    gpio_write(pi_handle, Config::PIN_M1_INA, 1);                // direction, while the driver is still disabled
    gpio_write(pi_handle, Config::PIN_M1_INB, 0);

    // 2) force the hardware PWM to 0 % BEFORE the pin is switched to the PWM function, then switch it and confirm 0 % again
    hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, 0);
    set_mode(pi_handle, Config::PIN_M1_PWM, PI_ALT0);
    if (hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, 0) != 0) {
        std::cerr << "[CRITICAL] Silicon PWM rejected on GPIO 13: the motor driver stays DISABLED.\n";
        return;                                                  // EN stays low: the motor cannot run
    }

    // 3) only now enable the driver
    gpio_write(pi_handle, Config::PIN_M1_EN, 1);
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
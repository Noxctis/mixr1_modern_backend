// src/motor.cpp
#include "motor.hpp"
#include "config.hpp"
#include <pigpiod_if2.h>
#include <iostream>
#include <algorithm>

void MotorController::park_fluid_outputs() {
    // Pump and solenoid valve OFF, exactly what act_wrapper.c park() does. Needed because a model that is killed (pkill, MATLAB crash,
    // SSH drop) never runs act_stop(): the pump enable or the valve could otherwise stay on. Pump first (its EN), then the valve:
    // its PWM pin is taken out of PWM mode and held low, then its direction pins low. The next model's act_init() sets them up again.
    // GPIO 13 / 17 / 27 (pump PWM + direction, shared with the mixer) are handled by init_pins() right after this.
    gpio_write(pi_handle, Config::PIN_FLUID_PUMP_EN, 0);
    set_mode(pi_handle, Config::PIN_FLUID_PUMP_EN, PI_OUTPUT);
    gpio_write(pi_handle, Config::PIN_FLUID_SOL_PWM, 0);
    set_mode(pi_handle, Config::PIN_FLUID_SOL_PWM, PI_OUTPUT);
    gpio_write(pi_handle, Config::PIN_FLUID_SOL_INA, 0);
    set_mode(pi_handle, Config::PIN_FLUID_SOL_INA, PI_OUTPUT);
    gpio_write(pi_handle, Config::PIN_FLUID_SOL_INB, 0);
    set_mode(pi_handle, Config::PIN_FLUID_SOL_INB, PI_OUTPUT);
}

void MotorController::init_pins() {
    park_fluid_outputs();                                        // the daemon owns the rig from here: pump and valve OFF first
    // SAFE START ORDER: the driver stays DISABLED (EN low) until the PWM output has been forced to 0 %.
    // pigpiod does NOT reset the hardware PWM when a program is killed or crashes, so a stale duty from an earlier run
    // would otherwise be applied the instant EN goes high (a burst of full power at start-up).
    // The same order is used after a MATLAB run: act_stop() leaves INA / INB low and the PWM clock set up by wiringPi.
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
        last_duty = -1;
        return;                                                  // EN stays low: the motor cannot run
    }
    last_duty = 0;

    // 3) only now enable the driver
    gpio_write(pi_handle, Config::PIN_M1_EN, 1);
}

MotorController::MotorController(int pi, bool start_released) : pi_handle(pi) {
    if (start_released) {
        set_mode(pi_handle, Config::PIN_M1_EN, PI_OUTPUT);
        gpio_write(pi_handle, Config::PIN_M1_EN, 0);             // mixer driver off; GPIO 13 / 17 / 27 belong to the running model
        released = true;
        return;
    }
    init_pins();
}

MotorController::~MotorController() {
    stop_motor();
}

void MotorController::set_pwm(int duty_cycle) {
    if (released) return;                                        // MATLAB owns GPIO 13: never write it
    duty_cycle = std::clamp(duty_cycle, 0, 4095);
    if (duty_cycle == last_duty) return;                         // unchanged: an idle daemon does not touch the PWM at all
    long long hw_duty = (static_cast<long long>(duty_cycle) * 1000000LL) / 4095LL;
    if (hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, static_cast<int>(hw_duty)) == 0) last_duty = duty_cycle;
    else last_duty = -1;                                         // try again next call
}

void MotorController::stop_motor() {
    if (!released) hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, 0);   // GPIO 13 only while it is ours
    gpio_write(pi_handle, Config::PIN_M1_EN, 0);
    last_duty = released ? -1 : 0;
}

void MotorController::release() {
    // EN low switches the mixer off at once. GPIO 13 is deliberately NOT written: the model has already set up the PWM for the pump
    // (wiringPi), and a pigpio hardware_PWM() call would re-program the shared PWM clock under it (pump AND solenoid on GPIO 12).
    gpio_write(pi_handle, Config::PIN_M1_EN, 0);
    released = true;
    last_duty = -1;
}

void MotorController::reclaim() {
    released = false;
    init_pins();
}

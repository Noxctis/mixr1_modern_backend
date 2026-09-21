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

    // NEW: Initialize ADS1115 on I2C bus 1
    adc_handle = i2c_open(pi_handle, 1, Config::I2C_ADC_ADDR, 0);
    if (adc_handle >= 0) {
        // Config: MUX=AIN0, PGA=+/-4.096V, Continuous Mode, 128SPS
        // ADS1115 expects MSB first, but SMBus sends LSB first. We swap 0xC283 to 0x83C2.
        i2c_write_word_data(pi_handle, adc_handle, 0x01, 0x83C2);
    } else {
        std::cerr << "[WARNING] Failed to connect to ADS1115 ADC at 0x48\n";
    }
}

MotorController::~MotorController() { 
    stop_motor(); 
    if (adc_handle >= 0) i2c_close(pi_handle, adc_handle); // Clean up
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

// NEW: Read actual current
double MotorController::get_current_amps() {
    if (adc_handle < 0) return 0.0;
    
    int val = i2c_read_word_data(pi_handle, adc_handle, 0x00);
    if (val < 0) return 0.0;
    
    // Swap bytes back: ADS1115 sends MSB first, SMBus reads LSB first
    val = ((val & 0xFF) << 8) | ((val >> 8) & 0xFF);
    int16_t raw_adc = static_cast<int16_t>(val);
    
    // +/- 4.096V range: 1 bit = 0.125mV = 0.000125V
    double voltage = raw_adc * 0.000125;
    double current = voltage / Config::VNH5019_CS_V_PER_AMP;
    
    return (current < 0.0) ? 0.0 : current; 
}

// NEW: Calculate power consumption
double MotorController::get_power_watts(int current_pwm) {
    double current = get_current_amps();
    double effective_voltage = Config::MOTOR_SUPPLY_VOLTAGE * (static_cast<double>(current_pwm) / 4095.0);
    return current * effective_voltage;
}
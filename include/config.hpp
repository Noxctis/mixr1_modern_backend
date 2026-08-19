// include/config.hpp
#pragma once
#include <cstddef>
#include <array>

namespace Config {
    // --- Kinematics & DSP ---
    constexpr double ENCODER_CPR = 192;               
    constexpr double RPM_ALPHA = 0.15;                   
    constexpr size_t SMA_WINDOW_SIZE = 8;                
    constexpr int DEADBAND_TICK_THRESHOLD = 2;           

    // --- GAIN SCHEDULING (Bisection Fluid Mapping) ---
    struct GainTier {
        double rpm_threshold; // Upper RPM limit for this tier
        double Kp;
        double Ki;
    };
    
    constexpr std::array<GainTier, 4> PI_SCHEDULE = {{
        {600.0,  1.5387, 40.3388},  // Low speed (0 to 600 RPM)
        {1100.0, 1.5195, 41.8610},  // Mid speed (601 to 1100 RPM)
        {1600.0, 1.4901, 37.8287},  // Mid-High (1101 to 1600 RPM)
        {2500.0, 1.4740, 40.1298}   // High speed (1601 to 2500 RPM)
    }};

    // --- Execution Pacing Matrix ---
    constexpr int RPM_SAMPLE_WINDOW_US = 10000;
    constexpr int LOOP_DELAY_US = 10000; 
    constexpr int NETWORK_PRESCALER = 1;
    constexpr int LCD_PRESCALER = 10;
    constexpr int SIMULINK_CHECK_INTERVAL = 100;         
    constexpr int TCP_PORT = 5000;                       

    // --- Hardware Pinout (BCM GPIO) ---
    constexpr unsigned int PIN_ENC_A = 24;               
    constexpr unsigned int PIN_ENC_B = 23;               
    constexpr unsigned int PIN_ENC_X = 22;               
    constexpr int ENCODER_DIRECTION = -1;                
    constexpr unsigned int PIN_M1_EN = 15;               
    constexpr unsigned int PIN_M1_INA = 17;              
    constexpr unsigned int PIN_M1_INB = 27;              
    constexpr unsigned int PIN_M1_PWM = 13;              
    constexpr int I2C_LCD_ADDR = 0x27;                   
    constexpr int PWM_FREQUENCY = 20000;                 
}
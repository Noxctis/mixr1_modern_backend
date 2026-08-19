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

    // --- GAIN SCHEDULING (Linear Interpolation Matrix) ---
    struct GainTier {
        double rpm; // Baseline RPM
        double Kp;
        double Ki;
    };
    
    constexpr std::array<GainTier, 5> PI_SCHEDULE = {{
        {0.00,    1.9646, 48.5934}, // Test 1: Stiction/Startup
        {342.81,  1.5387, 40.3388}, // Test 2: Low-Mid Speed
        {861.56,  1.5195, 41.8610}, // Test 3: Mid Speed
        {1378.44, 1.4901, 37.8287}, // Test 4: Mid-High Speed
        {1766.56, 1.4740, 40.1298}  // Test 5: Nominal Max Speed
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
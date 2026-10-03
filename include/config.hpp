// include/config.hpp
#pragma once
#include <cstddef>
#include <array>

namespace Config {
    enum class DisplayType {
        LCD1602,
        LCD2004,
        SSD1306_128x64
    };

    // --- Kinematics & DSP ---
    inline double ENCODER_CPR = 1024.0;               
    constexpr double RPM_ALPHA = 0.15;
    constexpr size_t SMA_WINDOW_SIZE = 8;                
    constexpr int DEADBAND_TICK_THRESHOLD = 2;           

// --- GLOBAL PI GAINS (1.0 rad/s Bandwidth) ---
    // inline (not constexpr) so they can be changed live (CMD:KP / CMD:KI) or via --kp= / --ki=.
    // Defaults below are the ORIGINAL tuning = the baseline. Update them after running tools/run_matrix.sh.
    inline double GLOBAL_KP = 1.5641;
    inline double GLOBAL_KI = 41.2249;

    // First-order low-pass on the PI *feedback* path:  y = a*x + (1-a)*y_prev
    // 1.0 = no filtering (baseline). Adviser suggested starting at 0.5 then tuning.
    // At the 10 ms loop: time constant ~ 10ms*(1-a)/a  (0.5->10ms, 0.3->23ms, 0.15->57ms)
    inline double FEEDBACK_ALPHA = 1.0;

    // --- Execution Pacing Matrix ---
    inline int RPM_SAMPLE_WINDOW_US = 20000;
    
    constexpr int LOOP_DELAY_US = 10000; 
    constexpr int NETWORK_PRESCALER = 1;
    constexpr int LCD_PRESCALER = 25;   // 25 x 10ms = 250ms refresh
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
    constexpr int I2C_OLED_ADDR = 0x3C;                  
    constexpr DisplayType DISPLAY_TYPE = DisplayType::SSD1306_128x64;
    constexpr int PWM_FREQUENCY = 20000;                 

    // --- EC11 (DFRobot Breakout) ---
    // Avoided: 5, 6, 13, 22, 23, 24 (already reserved)
    constexpr unsigned int PIN_EC11_A = 20;
    constexpr unsigned int PIN_EC11_B = 21;
    constexpr unsigned int PIN_EC11_SW = 26;
    constexpr int EC11_TRANSITIONS_PER_CLICK = 4;   // quadrature transitions per detent (set to 2 if your knob needs it)
    // Smooth turn-speed response (real-knob feel): the step per click grows continuously with click rate
    //   step = MIN + (MAX - MIN) * min(1, (rate / RATE_FULL)^GAMMA)
    constexpr double EC11_STEP_MIN_RPM   = 1.0;     // slowest turning: 1 RPM per click
    constexpr double EC11_STEP_MAX_RPM   = 25.0;    // fastest spinning: this many RPM per click
    constexpr double EC11_RATE_FULL_CPS  = 30.0;    // clicks/second at which the max step is reached
    constexpr double EC11_CURVE_GAMMA    = 1.6;     // >1 = more precision at low speed, ramps harder at high speed
    constexpr double EC11_RATE_SMOOTH    = 0.5;     // 0..1, higher = reacts faster, lower = smoother
    constexpr int    EC11_IDLE_RESET_MS  = 400;     // pause longer than this restarts at the fine step
    constexpr unsigned EC11_GLITCH_US = 300;         // hardware-timed debounce on A/B (raise if it double-counts, lower if fast spins are missed)
    constexpr unsigned EC11_BUTTON_DEBOUNCE_US = 30000; // ignore button edges closer than 30 ms

    // --- Standalone Control ---
    constexpr double STANDALONE_MIN_RPM = 0.0;
    constexpr double STANDALONE_MAX_RPM = 2500.0;
    constexpr double TORQUE_ESTIMATE_MAX_NM = 1.0;
}
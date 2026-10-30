// include/config.hpp
#pragma once
#include <cstddef>
#include <cstdint>
#include <array>

namespace Config {
    enum class DisplayType {
        LCD1602,
        LCD2004,
        SSD1306_128x64,
        ST7789_320x240      // 2.4" SPI TFT, landscape
    };

    // --- Kinematics & DSP ---
    inline double ENCODER_CPR = 1024.0;               
    constexpr double RPM_ALPHA = 0.15;
    constexpr size_t SMA_WINDOW_SIZE = 8;                
    constexpr int DEADBAND_TICK_THRESHOLD = 2;           

// --- GLOBAL PI GAINS (1.0 rad/s Bandwidth) ---
    // inline (not constexpr) so they can be changed live (CMD:KP / CMD:KI) or via --kp= / --ki=.
    // Defaults below are the ORIGINAL tuning = the baseline. Update them after running tools/run_matrix.sh.
    inline double GLOBAL_KP = 2.6294;
    inline double GLOBAL_KI = 12.0;

    // First-order low-pass on the PI *feedback* path:  y = a*x + (1-a)*y_prev
    // 1.0 = no filtering (baseline). Adviser suggested starting at 0.5 then tuning.
    // At the 10 ms loop: time constant ~ 10ms*(1-a)/a  (0.5->10ms, 0.3->23ms, 0.15->57ms)
    inline double FEEDBACK_ALPHA = 0.15;

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
    constexpr int ENCODER_DIRECTION = 1;                
    constexpr unsigned int PIN_M1_EN = 18;               
    constexpr unsigned int PIN_M1_INA = 27;              
    constexpr unsigned int PIN_M1_INB = 17;              
    constexpr unsigned int PIN_M1_PWM = 13;              
    // --- I2C buses (Linux /dev/i2c-N) ---
    // Each device is looked for on its preferred bus first, then on the other one, so the daemon also works if you move a device.
    constexpr int I2C_BUS_HARDWARE = 1;                  // the Pi's hardware I2C: GPIO 2 = SDA, GPIO 3 = SCL
    constexpr int I2C_BUS_EXTRA    = 3;                  // extra bus: GPIO 8 = SDA, GPIO 7 = SCL  (dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=8,i2c_gpio_scl=7)
    constexpr int I2C_BUS_DISPLAY  = I2C_BUS_HARDWARE;   // preferred bus of the OLED / LCD
    constexpr int I2C_BUS_POWER    = I2C_BUS_EXTRA;      // preferred bus of the wattmeter (INA219)
    constexpr int I2C_POWER_ADDR   = 0x45;               // wattmeter (DFRobot SEN0291 / INA219) address
    constexpr int POWER_READ_DIV   = 5;                  // daemon reads the wattmeter every N-th loop (5 -> 20 Hz): keeps the 100 Hz loop undisturbed
    constexpr int I2C_LCD_ADDR = 0x27;
    constexpr int I2C_OLED_ADDR = 0x3C;                  
    constexpr DisplayType DISPLAY_TYPE = DisplayType::ST7789_320x240;
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

    // --- Start key (module "KO" key) ---
    constexpr unsigned int PIN_START_KEY = 4;             // to GND when pressed, internal pull-up
    constexpr bool STANDALONE_RUN_AT_BOOT = false;        // false: the motor stays off in standalone mode until KO is pressed (KO toggles RUN / STOP)

    // --- ST7789 2.4" SPI TFT (pigpio hardware SPI0: SCL = GPIO 11, SDA/MOSI = GPIO 10) ---
    constexpr unsigned int PIN_TFT_DC  = 16;              // DC / RS
    constexpr unsigned int PIN_TFT_RES = 19;              // RES / RST   (GPIO 12 is the solenoid PWM in the fluid controller, so NOT used here)
    constexpr unsigned int PIN_TFT_CS  = 25;              // CS, driven by software and held low (the display is the only device on the bus)
    constexpr int      TFT_SPI_CHANNEL = 0;
    constexpr unsigned TFT_SPI_BAUD    = 20000000;        // lower to 10000000 if the picture is noisy on long wires
    constexpr int      TFT_SPI_MODE    = 0;               // try 3 if the screen stays blank
    constexpr uint8_t  TFT_MADCTL      = 0x60;            // landscape. mirrored / upside down: try 0xA0, 0x20, 0xE0.  Red and blue swapped: add 0x08
    constexpr bool     TFT_INVERT      = true;            // most ST7789 panels need INVON; set false if colours look inverted
    constexpr int      TFT_TEXT_SCALE  = 3;               // 6x8 font x3 = 18x24 px per character -> 17 columns x 10 rows
    constexpr uint16_t TFT_FG_RGB565   = 0xFFFF;          // white text
    constexpr uint16_t TFT_BG_RGB565   = 0x0000;          // black background

    // --- Standalone Control ---
    constexpr double STANDALONE_MIN_RPM = 0.0;
    constexpr double STANDALONE_MAX_RPM = 2500.0;
    constexpr double TORQUE_ESTIMATE_MAX_NM = 1.0;
}
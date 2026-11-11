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
    inline bool ESTIMATOR_DCT = false;                // --estimator=dct : the earlier discrete-time method (window = RPM_SAMPLE_WINDOW_US, set with --window=)
    inline bool ESTIMATOR_NEWEST = false;             // --estimator=newest : synchronous CET of the paper (newest edge, same-type start); default: aligned snapshots           

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
    constexpr int LCD_PRESCALER = 10;   // 25 x 10ms = 250ms refresh
    // MATLAB / Simulink detection (process_monitor.hpp): any <model>.elf the Simulink Pi package runs. Take-over within one scan,
    // hand-back only after the model has been gone this long (a Monitor & Tune restart does not bounce control back and forth).
    constexpr int MATLAB_SCAN_PERIOD_MS = 200;
    constexpr int MATLAB_RELEASE_DEBOUNCE_MS = 1000;
    constexpr int TCP_PORT = 5000;                       

    // --- Hardware Pinout (BCM GPIO) ---
    constexpr unsigned int PIN_ENC_A = 24;               
    constexpr unsigned int PIN_ENC_B = 23;               
    constexpr unsigned int PIN_ENC_X = 22;               
    constexpr int ENCODER_DIRECTION = -1;                
    constexpr unsigned int PIN_M1_EN = 18;               
    constexpr unsigned int PIN_M1_INA = 27;              
    constexpr unsigned int PIN_M1_INB = 17;              
    constexpr unsigned int PIN_M1_PWM = 13;
    // --- Fluid controller outputs (Simulink act_wrapper.c). The pump SHARES GPIO 13 / 17 / 27 with the mixer above. ---
    // The daemon never drives these; it only parks them OFF whenever it owns the rig (start-up and every hand-back from MATLAB),
    // so a model that was killed instead of stopped cannot leave the pump or the valve switched on.
    constexpr unsigned int PIN_FLUID_PUMP_EN = 14;
    constexpr unsigned int PIN_FLUID_SOL_INA = 5;
    constexpr unsigned int PIN_FLUID_SOL_INB = 6;
    constexpr unsigned int PIN_FLUID_SOL_PWM = 12;       // hardware PWM0 in the model; parked as a plain output, low
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
    // Knob step: slow turning = 1 RPM per detent, fast turning = jump to the next multiple of 10.
    // The gap is the time between detents; hysteresis stops it flickering between modes at borderline speed.
    constexpr double EC11_FAST_ENTER_MS = 50.0;    // detents closer together than this -> fast mode (about 14 detents/s)
    constexpr double EC11_FAST_EXIT_MS  = 120.0;   // detents further apart than this -> back to 1 RPM steps (keep > ENTER)
    constexpr unsigned EC11_GLITCH_US = 150;         // hardware-timed debounce on A/B (raise if it double-counts, lower if fast spins are missed)
    constexpr unsigned EC11_BUTTON_DEBOUNCE_US = 30000; // ignore button edges closer than 30 ms

    // --- Start key (module "KO" key) ---
    constexpr unsigned int PIN_START_KEY = 4;             // to GND when pressed, internal pull-up
    constexpr bool STANDALONE_RUN_AT_BOOT = false;        // false = Mode 1 starts STOPPED: turning the knob only sets the target, the KO key starts / stops the motor

    // --- Power safety cap (power_limiter.hpp) ---
    // The motor runs at most at POWER_LIMIT_W x POWER_LIMIT_AIM (72 W) and never goes above POWER_LIMIT_W (80 W): above the aim the PWM
    // ceiling is cut back automatically (Mode 1, Mode 2 and --test). Measured with the wattmeter (electrical input power); without a
    // wattmeter, torque x speed is used instead.
    constexpr double POWER_LIMIT_W = 90.0;               // hard ceiling: the power must never go above this
    constexpr double POWER_LIMIT_AIM = 0.90;             // the limiter regulates at 90 % of it = 80 W. The 10 W gap is room for the ~30 ms
                                                         // the wattmeter needs per reading, so short peaks (start-up, load changes) stay under 80 W
    constexpr double POWER_LIMIT_RECOVER_PCT_S = 10.0;   // ceiling released again by 10 % of full PWM per second once 5 % below the aim
    constexpr double POWER_TRIP_FACTOR = 1.0;            // above 100 % of the limit (90 W) ...
    constexpr int    POWER_TRIP_READINGS = 3;            // ... for 3 readings in a row (~0.1 s) -> motor OFF until the operator stops it
    constexpr double PWM_SLEW_PCT_S = 200.0;             // Mode 1 / 2: PWM may rise at most 200 % per second (0 -> 100 % in 0.5 s), so the
                                                         // power can never jump faster than the wattmeter (~30 readings/s) can see it
    // Stall trip: with the power capped, a jammed impeller would sit at the cap (72 W) forever. PWM above STALL_PWM_PCT while the shaft
    // turns slower than STALL_RPM for STALL_TIME_S -> motor OFF until the operator stops it (Mode 1, Mode 2 and --test).
    constexpr double STALL_PWM_PCT = 25.0;
    constexpr double STALL_RPM = 20.0;
    constexpr double STALL_TIME_S = 1.5;

    // --- Impeller check before every start in Mode 1 ---
    // The first KO press shows "IMPELLER 96 MM?"; a second KO press within this time starts the motor, a knob push cancels.
    constexpr int START_CONFIRM_TIMEOUT_MS = 10000;

    // --- Dashboard heartbeat ---
    // The dashboard should send "CMD:HB" (any line counts) every ~500 ms. Once the daemon has seen one, a dashboard that stays silent
    // for longer than this is dropped -> Mode 1, motor stopped. Dashboards that never send CMD:HB are still caught by the TCP timeouts below.
    constexpr int HEARTBEAT_TIMEOUT_MS = 2000;
    constexpr unsigned TCP_USER_TIMEOUT_MS = 3000;       // unacknowledged telemetry for this long (cable pulled) -> connection dropped

    // --- Torque zero before every experiment (amplifier zero command over RS485, same as the zero at boot) ---
    // The zero is normally taken WHILE IDLE: as soon as the shaft is at rest after a STOP (or after boot), so START is instant.
    // Only if START comes before that idle zero has finished is the motor held until it is done.
    // Every time the motor is started from standstill (Mode 1: KO key / knob up from 0, Mode 2: first non-zero RPM or PWM command)
    // the motor is held off, the daemon WAITS until the shaft has stopped, sends the amplifier's zero command, waits for its filter
    // to settle and reads the value back to check it really is ~0. If the amplifier refuses, the read-back is subtracted in software instead.
    constexpr int    TORQUE_TARE_READBACK   = 5;         // readings averaged for the before / after check
    constexpr int    TORQUE_TARE_ATTEMPTS   = 4;         // zero commands sent until the amplifier ITSELF reads 0.000 (last resort after that:
                                                         // software zero, with a warning)
    constexpr int    TORQUE_TARE_TIMEOUT_MS = 20000;     // whole zero procedure (4 attempts x up to 4 s); after that: start anyway
    constexpr double TORQUE_TARE_REST_S     = 0.5;       // the shaft must have been still this long before the zero is taken
    constexpr int    TORQUE_TARE_MIN_SETTLE_MS = 1000;   // after the zero command: read back at least this long (the amplifier's filter keeps
                                                         // sliding after the command, and can pass through 0 on the way to its final value)
    constexpr int    TORQUE_TARE_STABLE_READS = 20;      // ... then until 20 readings in a row (1 s) ...
    constexpr double TORQUE_TARE_STABLE_COUNTS = 1.0;    // ... agree within 1 count, AND the average of the 2nd half differs from the
    constexpr double TORQUE_TARE_CREEP_COUNTS = 0.2;     // 1st half by less than 0.2 count (no slow creep). Only that settled value is checked:
                                                         // it must read 0.000 (TORQUE_ZERO_EXACT_COUNTS), else the zero command is sent again.
    constexpr int    TORQUE_TARE_MAX_SETTLE_MS = 4000;   // give up waiting for stability after this (then the next attempt / software zero)
    constexpr double TORQUE_ZERO_EXACT_COUNTS = 0.5;     // the settled amplifier reading must round to 0.000 (|value| < half a count)
    constexpr double TORQUE_ZERO_CHECK_COUNTS = 0.5;     // before every start AND while idle at rest: the torque (0.5 s average) must read
                                                         // 0.000 (under half a count), else the amplifier is zeroed again (idle: max 3x per stop)
    constexpr int    TORQUE_IDLE_REZERO_MAX = 3;
    constexpr double TORQUE_TARE_REST_WAIT_S = 10.0;     // how long to wait for the shaft (and the fluid) to stop; then start WITHOUT a new zero

    // --- ST7789 2.4" SPI TFT (pigpio hardware SPI0: SCL = GPIO 11, SDA/MOSI = GPIO 10) ---
    constexpr unsigned int PIN_TFT_DC  = 16;              // DC / RS
    constexpr unsigned int PIN_TFT_RES = 19;              // RES / RST   (GPIO 12 is the solenoid PWM in the fluid controller, so NOT used here)
    constexpr unsigned int PIN_TFT_CS  = 25;              // CS, driven by software: low only while a transfer is in progress
    constexpr int      TFT_SPI_CHANNEL = 0;
    constexpr unsigned TFT_SPI_BAUD    = 20000000;        // lower to 10000000 if the picture is noisy on long wires
    constexpr int      TFT_SPI_MODE    = 3;               // try 3 if the screen stays blank
    constexpr uint8_t  TFT_MADCTL      = 0x60;            // landscape. mirrored / upside down: try 0xA0, 0x20, 0xE0.  Red and blue swapped: add 0x08
    constexpr bool     TFT_INVERT      = false;            // most ST7789 panels need INVON; set false if colours look inverted
    constexpr int      TFT_TEXT_SCALE  = 3;               // 6x8 font x3 = 18x24 px per character -> 17 columns x 10 rows
    constexpr uint16_t TFT_FG_RGB565   = 0xFFFF;          // white text
    constexpr uint16_t TFT_BG_RGB565   = 0x0000;          // black background

    // --- Standalone Control ---
    constexpr double STANDALONE_MIN_RPM = 0.0;
    constexpr double STANDALONE_MAX_RPM = 1500.0;

    // --- Impeller selection (menu at start-up; double-click the knob button to come back to it) ---
    struct ImpellerOption { const char* label; double max_rpm; };
    constexpr std::array<ImpellerOption, 2> IMPELLERS = {{
        {"96 MM",  1400.0},     // knob limit for the 96 mm impeller (covers the 1326 and 1362 RPM test speeds)
        {"174 MM",  350.0}      // knob limit for the 174 mm impeller
    }};
    constexpr int DOUBLE_CLICK_MS = 500;                // two presses of the knob button within this time = back to the impeller menu
    constexpr double TORQUE_ESTIMATE_MAX_NM = 1.0;
}
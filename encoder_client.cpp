/**
 * @file mixr1_daemon.cpp
 * @brief High-speed hardware telemetry and control daemon for the MIXR-1 chemical engineering platform.
 * @author Chrys Sean T. Sevilla, Cyril John Christian Calo, Sid Andre Bordario
 * @institution University of San Carlos - Computer Engineering Department
 * * @details 
 * This daemon operates as the primary hardware abstraction layer (HAL) for the MIXR-1 plant.
 * It executes in user-space via the pigpiod_if2 client library, ensuring non-blocking DMA access
 * to Broadcom silicon peripherals. The architecture is explicitly decoupled to allow seamless 
 * hardware handovers to external control processes (e.g., MATLAB/Simulink Real-Time).
 */

#include <iostream>
#include <string>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <chrono>
#include <algorithm>
#include <iomanip>
#include <sstream>
#include <array>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>

// ==========================================
// 1. GLOBAL CONFIGURATION (INDUSTRY STANDARD)
// ==========================================
// Best Practice: Centralize all magic numbers into a discrete compile-time namespace. 
// This prevents silent mathematical errors and allows rapid recalibration without hunting through logic.
namespace Config {
    // --- Kinematics & DSP ---
    constexpr double ENCODER_CPR = 617.35;               ///< Counts per revolution (12 pulses * 51.446 gear ratio)
    constexpr double RPM_ALPHA = 0.15;                   ///< EMA Smoothing coefficient (~2.4Hz Low-Pass Cutoff at 100Hz loop)
    constexpr size_t SMA_WINDOW_SIZE = 8;                ///< UI Rolling average (8 ticks @ 10Hz = 800ms to match LCD tachometer gate)
    constexpr int DEADBAND_TICK_THRESHOLD = 2;           ///< Rejects +/- 1-tick quantization noise dithering when PWM is 0

    // --- Networking & Execution Timing ---
    constexpr int TCP_PORT = 5000;                       ///< Binding port for the Python Dashboard / GUI
    constexpr int LOOP_DELAY_US = 10000;                 ///< 10ms target loop pacing (~100Hz discrete execution)
    constexpr int TELEMETRY_PRESCALER = 10;              // Decouples 10Hz network/LCD transmission from the 100Hz control loop
    constexpr int SIMULINK_CHECK_INTERVAL = 100;         // Polls the OS process list every 1 second (100 ticks)

    // --- Hardware Pinout & Peripherals (BCM GPIO) ---
    constexpr unsigned int PIN_ENC_A = 23;               ///< Quadrature Channel A (Interrupt driven)
    constexpr unsigned int PIN_ENC_B = 24;               ///< Quadrature Channel B (Interrupt driven)
    constexpr unsigned int PIN_M1_EN = 15;               ///< VNH5019 Enable
    constexpr unsigned int PIN_M1_INA = 17;              ///< VNH5019 Direction A
    constexpr unsigned int PIN_M1_INB = 27;              ///< VNH5019 Direction B
    constexpr unsigned int PIN_M1_PWM = 13;              ///< PI_ALT0 Hardware PWM Silicon Output
    constexpr int I2C_LCD_ADDR = 0x27;                   ///< 16x2 Display Backpack Address
    constexpr int PWM_FREQUENCY = 20000;                 ///< 20kHz Carrier Frequency (Acoustically inaudible, optimal for VNH5019)
}

// Global run flag for graceful daemon termination via POSIX signals
std::atomic<bool> run_loop{true};

/**
 * @brief Interrupt service routine for OS signals (Ctrl+C, SIGTERM).
 * Safely breaks the main loop, ensuring motor logic and socket bindings are closed cleanly.
 */
void signal_handler(int signum) {
    run_loop = false;
}

// ==========================================
// 2. SYSTEM MODULE: PROCESS MONITOR
// ==========================================
class ProcessMonitor {
public:
    /**
     * @brief Detects the presence of MATLAB/Simulink standalone executables.
     * @return true if the process is currently active in the Linux scheduler.
     * @note Uses regex `[r]` to prevent the `pgrep` command itself from triggering a false positive.
     */
    [[nodiscard]] static bool is_simulink_running() {
        const char* cmd = "pgrep -x \"[r]aspberrypi_get\" > /dev/null 2>&1";
        return (std::system(cmd) == 0);
    }
};

// ==========================================
// 3. DSP & KINEMATICS ENGINE
// ==========================================
/**
 * @class KinematicsEngine
 * @brief Encapsulates all temporal and spatial mathematics.
 * Best Practice: The Single Responsibility Principle dictates that the main execution loop 
 * should not manage clock generation or filter states. This engine ingests raw tick counts 
 * and outputs clean, filtered RPM structs.
 */
class KinematicsEngine {
private:
    std::chrono::time_point<std::chrono::high_resolution_clock> last_time;
    long long last_count = 0;
    
    double ema_rpm = 0.0;
    std::array<double, Config::SMA_WINDOW_SIZE> sma_history{};
    size_t sma_index = 0;
    double sma_sum = 0.0;
    size_t sma_count = 0;

public:
    /// Standardized payload structure containing mathematical derivatives
    struct TelemetryState {
        double exact_rpm;           ///< Unfiltered instantaneous velocity
        double ema_filtered_rpm;    ///< High-speed hardware filtered velocity (Mode 3 loop payload)
        double sma_ui_rpm;          ///< Time-integrated velocity (Dashboard presentation payload)
    };

    /**
     * @brief Resets the chronometric state and flushes filter buffers.
     * Must be called immediately upon claiming hardware to prevent massive delta-time spikes.
     * @param current_encoder_count The baseline hardware tick value at t=0.
     */
    void reset(long long current_encoder_count) {
        last_time = std::chrono::high_resolution_clock::now();
        last_count = current_encoder_count;
        ema_rpm = 0.0;
        sma_sum = 0.0;
        sma_index = 0;
        sma_count = 0;
        sma_history.fill(0.0);
    }

    /**
     * @brief Core exact-time derivative calculation loop.
     * @param current_count The absolute quadrature tick reading from the DMA interrupt.
     * @param current_pwm Required to evaluate the mechanical deadband noise gate.
     * @param update_sma Flag to trigger the rolling presentation average (runs at 1/10th frequency).
     */
    [[nodiscard]] TelemetryState process(long long current_count, int current_pwm, bool update_sma) {
        TelemetryState state{0.0, 0.0, 0.0};

        // 1. Exact-Time Generation
        // Using C++ chrono prevents integer rollover errors common in POSIX struct timespec
        auto current_time = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> dt_sec = current_time - last_time;
        last_time = current_time;

        long long delta_ticks = current_count - last_count;
        last_count = current_count;

        // 2. Velocity Math & Deadband Gate
        if (dt_sec.count() > 0.0) {
            if (current_pwm == 0 && std::abs(delta_ticks) <= Config::DEADBAND_TICK_THRESHOLD) {
                state.exact_rpm = 0.0; // Suppress mechanical vibration dithering
            } else {
                state.exact_rpm = (static_cast<double>(delta_ticks) / Config::ENCODER_CPR) * (60.0 / dt_sec.count());
            }
        }

        // 3. Exponential Moving Average (IIR Hardware Filter)
        // Eliminates digital quantization noise before transmission
        ema_rpm = (Config::RPM_ALPHA * state.exact_rpm) + ((1.0 - Config::RPM_ALPHA) * ema_rpm);
        state.ema_filtered_rpm = ema_rpm;

        // 4. Simple Moving Average (Presentation Filter)
        // Replicates the 0.8s integration window of the physical optical tachometer
        if (update_sma) {
            sma_sum -= sma_history[sma_index];
            sma_history[sma_index] = ema_rpm;
            sma_sum += sma_history[sma_index];
            
            sma_index = (sma_index + 1) % Config::SMA_WINDOW_SIZE;
            if (sma_count < Config::SMA_WINDOW_SIZE) sma_count++;
        }
        
        state.sma_ui_rpm = (sma_count == 0) ? 0.0 : (sma_sum / static_cast<double>(sma_count));
        return state;
    }
};

// ==========================================
// 4. HARDWARE MODULE: QUADRATURE ENCODER
// ==========================================
class PololuEncoder {
private:
    int pi_handle;
    unsigned int pin_a, pin_b;
    int cb_a, cb_b;
    
    // Best Practice: `std::atomic` ensures thread-safe, lock-free memory access across 
    // the boundary between the main execution thread and the pigpio internal interrupt thread.
    std::atomic<long long> count{0};
    uint8_t state = 0;
    uint8_t val_a = 0;
    uint8_t val_b = 0;
    
    // Valid 4-edge transition matrix mapping (+1 forward, -1 reverse, 0 invalid noise)
    static constexpr int QUAD_STATES[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    /**
     * @brief Static C-style callback router required by the pigpio C API.
     */
    static void isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
        if (level > 1) return; // Ignore DMA watchdog timeout ticks
        static_cast<PololuEncoder*>(user)->update_state(gpio, level);
    }

    /**
     * @brief Zero-latency state decoder.
     * Processes the specific logic level pushed directly by the DMA engine.
     */
    void update_state(unsigned gpio, unsigned level) {
        if (gpio == pin_a) val_a = level;
        else if (gpio == pin_b) val_b = level;

        state = (state << 2) & 0x0F;
        state |= (val_a << 1) | val_b;
        
        // Use memory_order_relaxed for maximum performance, as strict ordering is not required for a simple counter
        count.fetch_add(QUAD_STATES[state], std::memory_order_relaxed);
    }

public:
    PololuEncoder(int pi, unsigned int a, unsigned int b) : pi_handle(pi), pin_a(a), pin_b(b) {
        set_mode(pi_handle, pin_a, PI_INPUT);
        set_mode(pi_handle, pin_b, PI_INPUT);
        set_pull_up_down(pi_handle, pin_a, PI_PUD_UP);
        set_pull_up_down(pi_handle, pin_b, PI_PUD_UP);
        
        // Fetch baseline to initialize the phase tracker alignment
        val_a = gpio_read(pi_handle, pin_a);
        val_b = gpio_read(pi_handle, pin_b);
        
        cb_a = callback_ex(pi_handle, pin_a, EITHER_EDGE, isr_router, this);
        cb_b = callback_ex(pi_handle, pin_b, EITHER_EDGE, isr_router, this);
    }

    ~PololuEncoder() {
        callback_cancel(cb_a);
        callback_cancel(cb_b);
    }

    [[nodiscard]] long long get_count() const {
        return count.load(std::memory_order_relaxed);
    }
};

// ==========================================
// 5. HARDWARE MODULE: MOTOR CONTROLLER
// ==========================================
class MotorController {
private:
    int pi_handle;

public:
    explicit MotorController(int pi) : pi_handle(pi) {
        set_mode(pi_handle, Config::PIN_M1_EN, PI_OUTPUT);
        set_mode(pi_handle, Config::PIN_M1_INA, PI_OUTPUT);
        set_mode(pi_handle, Config::PIN_M1_INB, PI_OUTPUT);
        
        // Enforce PI_ALT0 to map GPIO 13 to the internal BCM silicon clock
        set_mode(pi_handle, Config::PIN_M1_PWM, PI_ALT0);

        gpio_write(pi_handle, Config::PIN_M1_EN, 1);
        gpio_write(pi_handle, Config::PIN_M1_INA, 1);
        gpio_write(pi_handle, Config::PIN_M1_INB, 0); // Forward locked
        
        if (hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, 0) != 0) {
            std::cerr << "[CRITICAL] Silicon PWM rejected on GPIO 13\n";
        }
    }

    ~MotorController() {
        stop_motor(); // Inherent fail-safe via RAII destruction
    }

    /**
     * @brief Translates 8-bit UI commands to 20-bit silicon resolution.
     */
    void set_pwm(int duty_cycle) {
        duty_cycle = std::clamp(duty_cycle, 0, 255);
        int hw_duty = (duty_cycle * 1000000) / 255;
        hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, hw_duty);
    }

    void stop_motor() {
        hardware_PWM(pi_handle, Config::PIN_M1_PWM, Config::PWM_FREQUENCY, 0);
        gpio_write(pi_handle, Config::PIN_M1_EN, 0);
    }
};

// ==========================================
// 6. HARDWARE MODULE: LCD DISPLAY 1602
// ==========================================
class LCD1602 {
private:
    int pi_handle;
    int i2c_handle;
    int addr;
    int backlight = 0x08;

    void write_byte(int val) {
        i2c_write_byte(pi_handle, i2c_handle, val | backlight);
    }

    void toggle_enable(int val) {
        write_byte(val | 0x04);
        usleep(50); // Minimum enable pulse width constraint
        write_byte(val & ~0x04);
        usleep(50);
    }

    void send_command(int cmd) {
        int high = cmd & 0xF0;
        int low = (cmd << 4) & 0xF0;
        write_byte(high); toggle_enable(high);
        write_byte(low); toggle_enable(low);
    }

    void send_data(int data) {
        int high = data & 0xF0;
        int low = (data << 4) & 0xF0;
        write_byte(high | 0x01); toggle_enable(high | 0x01);
        write_byte(low | 0x01); toggle_enable(low | 0x01);
    }

public:
    LCD1602(int pi, int i2c_addr = Config::I2C_LCD_ADDR) : pi_handle(pi), addr(i2c_addr) {
        i2c_handle = i2c_open(pi_handle, 1, addr, 0);
        if (i2c_handle < 0) return;

        // HD44780 standard 4-bit initialization sequence
        usleep(50000);
        for (int i = 0; i < 3; ++i) {
            write_byte(0x30); toggle_enable(0x30);
            usleep((i == 0) ? 5000 : 150);
        }

        write_byte(0x20); toggle_enable(0x20); // Switch to 4-bit
        send_command(0x28); // 4-bit, 2-line, 5x8
        send_command(0x0C); // Display ON, Cursor OFF
        send_command(0x06); // Increment mode
        clear();
    }

    ~LCD1602() {
        if (i2c_handle >= 0) {
            clear();
            i2c_close(pi_handle, i2c_handle);
        }
    }

    void clear() {
        send_command(0x01);
        usleep(2000); // Display clear requires extended execution time
    }

    void set_cursor(int row, int col) {
        int row_offsets[] = { 0x00, 0x40 };
        send_command(0x80 | (col + row_offsets[row]));
    }

    void print(const std::string& str) {
        for (char c : str) send_data(c);
    }
};

// ==========================================
// 7. NETWORK MODULE: TCP SERVER
// ==========================================
class TelemetryServer {
private:
    int server_fd = -1;
    int client_socket = -1;
    std::string rx_buffer;

    void disconnect_client() {
        if (client_socket >= 0) close(client_socket);
        client_socket = -1;
    }

public:
    ~TelemetryServer() { stop_server(); }

    [[nodiscard]] bool start_server(int port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) return false;
        
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) return false;
        return listen(server_fd, 1) >= 0;
    }

    /**
     * @brief Non-blocking client acceptor loop.
     * Prevents the daemon from hard-locking during boot if the GUI is not yet active.
     */
    [[nodiscard]] bool wait_for_client() {
        std::cout << "[MIXR-1] Waiting for Dashboard (Port " << Config::TCP_PORT << ")...\n";
        while (run_loop) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(server_fd, &readfds);
            struct timeval tv{1, 0}; // 1-second timeout
            
            if (select(server_fd + 1, &readfds, nullptr, nullptr, &tv) > 0) {
                if (FD_ISSET(server_fd, &readfds)) {
                    client_socket = accept(server_fd, nullptr, nullptr);
                    
                    // TCP_NODELAY: Explicitly defeats Nagle's Algorithm to prevent buffering latency
                    int flag = 1;
                    setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
                    rx_buffer.clear();
                    return client_socket >= 0;
                }
            }
        }
        return false;
    }

    [[nodiscard]] bool send_packet(double rpm, double torque) const {
        if (client_socket < 0) return false;
        std::string packet = std::to_string(rpm) + "," + std::to_string(torque) + "\n";
        return send(client_socket, packet.c_str(), packet.length(), MSG_NOSIGNAL) > 0;
    }

    /**
     * @brief Drains the kernel socket instantly.
     * @details Uses MSG_DONTWAIT to exhaust the buffer, extracting only the most recent 
     * valid command packet. This prevents UI slider backlog accumulation.
     */
    [[nodiscard]] bool receive_command(int& new_pwm) {
        if (client_socket < 0) return false;
        bool updated = false;
        char chunk[1024];

        while (true) {
            ssize_t bytes = recv(client_socket, chunk, sizeof(chunk) - 1, MSG_DONTWAIT);
            if (bytes > 0) {
                chunk[bytes] = '\0';
                rx_buffer += chunk;
            } else {
                break;
            }
        }

        size_t pos;
        while ((pos = rx_buffer.find('\n')) != std::string::npos) {
            std::string line = rx_buffer.substr(0, pos);
            rx_buffer.erase(0, pos + 1);

            size_t cmd_pos = line.find("CMD:PWM,");
            if (cmd_pos != std::string::npos) {
                try {
                    new_pwm = std::stoi(line.substr(cmd_pos + 8));
                    updated = true;
                } catch (...) {}
            }
        }
        return updated;
    }

    void stop_server() {
        disconnect_client();
        if (server_fd >= 0) close(server_fd);
        server_fd = -1;
    }
};

// ==========================================
// 8. MAIN DAEMON ORCHESTRATOR
// ==========================================
/**
 * @brief The primary state machine. 
 * Responsibilities are strictly limited to object lifecycle management and hardware 
 * handover interlocks. All math and state tracking is handled by the respective classes.
 */
int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGPIPE, SIG_IGN); 

    auto network = std::make_unique<TelemetryServer>();
    KinematicsEngine kinematics;

    while (run_loop) {
        if (!network->start_server(Config::TCP_PORT)) {
            std::cerr << "CRITICAL: Port locked. Retrying...\n";
            usleep(2000000);
            continue;
        }

        if (network->wait_for_client()) {
            std::cout << "[MIXR-1] Dashboard Connected.\n";
            
            int pi = -1;
            std::unique_ptr<PololuEncoder> encoder = nullptr;
            std::unique_ptr<MotorController> motor = nullptr;
            std::unique_ptr<LCD1602> lcd = nullptr;
            
            bool mode3_notified = false;
            int current_pwm = 0;
            int simulink_check_counter = Config::SIMULINK_CHECK_INTERVAL; 
            int telemetry_prescaler = 0;
            bool simulink_is_active = false;

            while (run_loop) {
                // 1. Process Monitor Interlock
                if (++simulink_check_counter >= Config::SIMULINK_CHECK_INTERVAL) {
                    simulink_check_counter = 0;
                    simulink_is_active = ProcessMonitor::is_simulink_running();
                }

                // 2. MATLAB Hardware Handover (Mode 3)
                // Safely releases memory pointers and hardware API bindings so Simulink can attach.
                if (simulink_is_active) {
                    if (!mode3_notified) {
                        std::cout << "[MIXR-1] MATLAB detected. Releasing hardware...\n";
                        motor.reset();
                        encoder.reset();
                        lcd.reset();
                        if (pi >= 0) { pigpio_stop(pi); pi = -1; } 
                        mode3_notified = true;
                    }
                    // Transmit a negative sentinel value to visually lock the Python UI
                    if (!network->send_packet(-2.0, -2.0)) break; 
                    usleep(1000000); 
                    continue;
                }

                if (mode3_notified) {
                    std::cout << "[MIXR-1] MATLAB teardown complete.\n";
                    mode3_notified = false;
                }

                // 3. Daemon Hardware Re-Claim (Mode 2)
                if (pi < 0) {
                    pi = pigpio_start(nullptr, nullptr);
                    if (pi >= 0) {
                        encoder = std::make_unique<PololuEncoder>(pi, Config::PIN_ENC_A, Config::PIN_ENC_B);
                        motor = std::make_unique<MotorController>(pi);
                        lcd = std::make_unique<LCD1602>(pi);
                        
                        // Seed the engine immediately upon hardware connection
                        kinematics.reset(encoder->get_count());
                    } else {
                        usleep(2000000);
                        continue;
                    }
                }

                // 4. Command Execution
                if (network->receive_command(current_pwm) && motor) {
                    motor->set_pwm(current_pwm);
                }

                // 5. Engine Execution & Mathematics
                bool update_ui = (++telemetry_prescaler >= Config::TELEMETRY_PRESCALER);
                if (update_ui) telemetry_prescaler = 0;

                // Main loop passes raw data in, receives a complete telemetry state block out
                auto state = kinematics.process(encoder->get_count(), current_pwm, update_ui);

                // 6. Network TX & Presentation
                if (update_ui) {
                    if (!network->send_packet(state.ema_filtered_rpm, 0.0)) break; 
                    
                    if (lcd) {
                        std::ostringstream r_str, p_str;
                        r_str << std::fixed << std::setprecision(1) << "RPM: " << state.sma_ui_rpm << "   ";
                        p_str << "PWM: " << current_pwm << "   ";
                        lcd->set_cursor(0, 0); lcd->print(r_str.str());
                        lcd->set_cursor(1, 0); lcd->print(p_str.str());
                    }
                }
                
                // Base loop pacing (~100Hz constraint)
                usleep(Config::LOOP_DELAY_US); 
            }

            // Fallback teardown on loop break
            motor.reset();
            encoder.reset();
            lcd.reset();
            if (pi >= 0) pigpio_stop(pi);
        }
        network->stop_server();
    }

    std::cout << "\n[MIXR-1] Daemon safely offline.\n";
    return 0;
}
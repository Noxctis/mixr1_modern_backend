/**
 * @file mixr1_daemon.cpp
 * @brief High-speed hardware telemetry and control daemon for the MIXR-1 chemical engineering platform.
 * @author Chrys Sean T. Sevilla, Cyril John Christian Calo, Sid Andre Bordario
 * @institution University of San Carlos - Computer Engineering Department
 * * @architecture
 * Operates in User Space via the pigpiod_if2 client library. This architecture allows
 * safe hardware handovers to external processes (MATLAB/Simulink) without triggering 
 * kernel-level GPIO locks.
 * * @features
 * 1. Exact-time differential math using high-resolution chrono clocks.
 * 2. Exponential Moving Average (EMA) filtering for quantization noise attenuation.
 * 3. Zero-network-trip encoder state tracking via DMA interrupt payloads.
 * 4. Silicon-level Hardware PWM locked at 20kHz for the VNH5019 driver.
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
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h> 

// ==========================================
// GLOBAL CONFIGURATION & CONSTANTS
// ==========================================

/** * @brief TEST BENCH CONFIG: Pololu Micro Metal Gearmotor
 * Calculation: 12 base magnetic pulses * 51.446 gearbox ratio = 617.35 counts per revolution.
 */
constexpr double ENCODER_CPR = 617.35; 

constexpr int TCP_PORT = 5000;

/**
 * @brief Establishes the baseline ~100Hz execution frequency of the main control loop.
 * 10,000 microseconds = 10ms. 
 */
constexpr int LOOP_DELAY_US = 10000; 

std::atomic<bool> run_loop(true);

/**
 * @brief Safely interrupts the daemon loop to release hardware and network bindings on SIGINT/SIGTERM.
 */
void signal_handler(int signum) {
    run_loop = false;
}

// ==========================================
// SYSTEM MODULE: PROCESS MONITOR
// ==========================================
class ProcessMonitor {
public:
    /**
     * @brief Detects external MATLAB/Simulink hardware claims.
     * Scans the Linux process table for the truncated Simulink binary. 
     * Regex brackets prevent matching the grep shell command itself.
     * @return true if Simulink is actively running.
     */
    static bool is_simulink_running() {
        const char* cmd = "pgrep -x \"[r]aspberrypi_get\" > /dev/null 2>&1";
        return (system(cmd) == 0);
    }
};

// ==========================================
// HARDWARE MODULE: QUADRATURE ENCODER
// ==========================================
class PololuEncoder {
private:
    int pi_handle;
    unsigned int pin_a, pin_b;
    int cb_a, cb_b;
    volatile long long count = 0;
    
    // Internal cache isolates state tracking from the network stack to prevent latency
    volatile uint8_t val_a = 0;
    volatile uint8_t val_b = 0;
    volatile uint8_t state = 0;
    
    // Standard quadrature transition matrix mapping 4-edge states (+1, -1, or 0 for invalid/noise)
    const int QUAD_STATES[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    /**
     * @brief Static C-style callback router required for pigpio DMA interrupts.
     */
    static void isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
        if (level > 1) return; // Level 2 indicates a DMA watchdog timeout. Discard to prevent math corruption.
        static_cast<PololuEncoder*>(user)->update_state(gpio, level);
    }

    /**
     * @brief Processes physical encoder ticks.
     * ZERO-NETWORK-TRIP ARCHITECTURE: Utilizes the exact level parameter pushed directly by the 
     * daemon's DMA interrupt, eliminating the need to poll the daemon over the localhost socket. 
     * This physically prevents data-dropping and phase-shifting at high velocities.
     */
    void update_state(unsigned gpio, unsigned level) {
        if (gpio == pin_a) val_a = level;
        else if (gpio == pin_b) val_b = level;

        state = (state << 2) & 0x0F;
        state |= (val_a << 1) | val_b;
        count += QUAD_STATES[state];
    }

public:
    PololuEncoder(int pi, unsigned int a, unsigned int b) : pi_handle(pi), pin_a(a), pin_b(b) {
        set_mode(pi_handle, pin_a, PI_INPUT);
        set_mode(pi_handle, pin_b, PI_INPUT);
        set_pull_up_down(pi_handle, pin_a, PI_PUD_UP);
        set_pull_up_down(pi_handle, pin_b, PI_PUD_UP);
        
        // Fetch baseline states on instantiation to initialize the phase tracker alignment
        val_a = gpio_read(pi_handle, pin_a);
        val_b = gpio_read(pi_handle, pin_b);
        
        cb_a = callback_ex(pi_handle, pin_a, EITHER_EDGE, isr_router, this);
        cb_b = callback_ex(pi_handle, pin_b, EITHER_EDGE, isr_router, this);
    }

    ~PololuEncoder() {
        callback_cancel(cb_a);
        callback_cancel(cb_b);
    }

    long long get_count() const { return count; }
};

// ==========================================
// HARDWARE MODULE: MOTOR CONTROLLER (VNH5019)
// ==========================================
class MotorController {
private:
    int pi_handle;
    const unsigned int M1EN = 15;
    const unsigned int M1INA = 17;
    const unsigned int M1PWM = 13; // Broadcom Silicon Hardware PWM Channel 1
    const unsigned int M1INB = 27;

public:
    MotorController(int pi) : pi_handle(pi) {
        set_mode(pi_handle, M1EN, PI_OUTPUT);
        set_mode(pi_handle, M1INA, PI_OUTPUT);
        set_mode(pi_handle, M1INB, PI_OUTPUT);
        
        // PI_ALT0 explicitly maps GPIO 13 to the BCM internal silicon PWM generator
        set_mode(pi_handle, M1PWM, PI_ALT0);

        // VNH5019 Forward Logic Topology
        gpio_write(pi_handle, M1EN, 1);
        gpio_write(pi_handle, M1INA, 1);
        gpio_write(pi_handle, M1INB, 0);
        
        /**
         * SILICON HARDWARE PWM 
         * Bypasses OS software DMA scheduling limits. 
         * Frequency locked to 20,000 Hz: Pushes the acoustic carrier frequency up to the 
         * theoretical ceiling of the STMicroelectronics VNH5019 chip.
         */
        int status = hardware_PWM(pi_handle, M1PWM, 20000, 0); 
        if (status != 0) {
            std::cerr << "[CRITICAL HARDWARE FAULT] Silicon PWM rejected on GPIO 13. Code: " << status << std::endl;
        }
    }

    ~MotorController() {
        stop_motor();
    }

    /**
     * @brief Translates 8-bit UI commands to 20-bit silicon resolution.
     * @param duty_cycle Standard 0-255 scale.
     */
    void set_pwm(int duty_cycle) {
        if (duty_cycle < 0) duty_cycle = 0;
        if (duty_cycle > 255) duty_cycle = 255;
        
        // hardware_PWM function requires duty cycle mapped to a 1,000,000 base integer
        int hw_duty = (duty_cycle * 1000000) / 255;
        
        int status = hardware_PWM(pi_handle, M1PWM, 20000, hw_duty);
        if (status != 0) {
            std::cerr << "[PWM ERROR] Failed to update duty cycle. Code: " << status << std::endl;
        }
    }

    void stop_motor() {
        hardware_PWM(pi_handle, M1PWM, 20000, 0);
        gpio_write(pi_handle, M1EN, 0); 
    }
};

// ==========================================
// NETWORK MODULE: TCP SERVER
// ==========================================
class TelemetryServer {
private:
    int server_fd = -1;
    int client_socket = -1;
    std::string rx_buffer = ""; 

public:
    ~TelemetryServer() { stop_server(); }

    bool start_server(int port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        if (server_fd < 0) return false;
        
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) return false;
        if (listen(server_fd, 1) < 0) return false;
        return true;
    }

    bool wait_for_client() {
        std::cout << "[MIXR-1] Mode 2 Active. Waiting for Dashboard (Port " << TCP_PORT << ")..." << std::endl;
        while (run_loop) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(server_fd, &readfds);
            struct timeval tv{1, 0}; 
            int activity = select(server_fd + 1, &readfds, nullptr, nullptr, &tv);

            if (activity > 0 && FD_ISSET(server_fd, &readfds)) {
                client_socket = accept(server_fd, nullptr, nullptr);
                
                // IPPROTO_TCP / TCP_NODELAY: Explicitly disables Nagle's Algorithm.
                // Prevents kernel-level packet buffering to guarantee zero-latency telemetry streaming.
                int flag = 1;
                setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, (char *)&flag, sizeof(int));
                
                rx_buffer.clear(); 
                return (client_socket >= 0);
            }
        }
        return false; 
    }

    bool send_packet(double rpm, double torque) {
        if (client_socket < 0) return false;
        std::string packet = std::to_string(rpm) + "," + std::to_string(torque) + "\n";
        ssize_t sent = send(client_socket, packet.c_str(), packet.length(), MSG_NOSIGNAL);
        return (sent > 0);
    }

    /**
     * @brief Drains the incoming kernel socket instantly.
     * Employs MSG_DONTWAIT to exhaust the buffer, extracting only the most recent complete command 
     * packet. Prevents UI slider backlog accumulation and resulting hardware desynchronization.
     * @param new_pwm Passed by reference. Updated only if a valid command is parsed.
     */
    bool receive_command(int& new_pwm) {
        if (client_socket < 0) return false;
        bool command_updated = false;
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
                    command_updated = true;
                } catch (...) {}
            }
        }
        
        return command_updated;
    }

    void disconnect_client() {
        if (client_socket >= 0) close(client_socket);
        client_socket = -1;
    }

    void stop_server() {
        disconnect_client();
        if (server_fd >= 0) close(server_fd);
        server_fd = -1;
    }
};

// ==========================================
// MAIN DAEMON LOOP
// ==========================================
int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGPIPE, SIG_IGN); 

    auto network = std::make_unique<TelemetryServer>();

    while (run_loop) {
        if (!network->start_server(TCP_PORT)) {
            std::cerr << "CRITICAL: Port " << TCP_PORT << " locked. Retrying..." << std::endl;
            usleep(2000000);
            continue;
        }

        if (network->wait_for_client()) {
            std::cout << "[MIXR-1] Dashboard Connected. Managing state engine..." << std::endl;
            
            int pi = -1;
            std::unique_ptr<PololuEncoder> encoder = nullptr;
            std::unique_ptr<MotorController> motor = nullptr;
            
            bool mode3_notified = false;
            long long last_count = 0;
            int current_pwm = 0;

            int simulink_check_counter = 100; 
            int telemetry_prescaler = 0;
            bool simulink_is_active = false;

            // DIGITAL FILTER VARIABLES
            double filtered_rpm = 0.0;
            
            /** * @brief Exponential Moving Average (EMA) Tuning Coefficient.
             * 0.15 establishes a low-pass filter cutoff frequency of ~2.4 Hz at a 100Hz loop rate.
             * Blocks digital quantization noise generated by integer tick limits while preserving 
             * the true physical momentum of the fluid mechanics.
             */
            const double RPM_ALPHA = 0.15; 

            // High-resolution clock tracker initialized for Exact-Time math
            auto last_time = std::chrono::high_resolution_clock::now();

            while (run_loop) {
                // 1. Process Monitor Matrix (Checked at 1Hz to conserve CPU cycles)
                if (++simulink_check_counter >= 100) {
                    simulink_check_counter = 0;
                    simulink_is_active = ProcessMonitor::is_simulink_running();
                }

                // 2. Mode 3 Handover Execution (MATLAB Process Control)
                if (simulink_is_active) {
                    if (!mode3_notified) {
                        std::cout << "\n[MIXR-1] MATLAB detected. Releasing hardware pins..." << std::endl;
                        motor.reset();
                        encoder.reset();
                        // Releases the daemon socket binding so Simulink can claim it
                        if (pi >= 0) { pigpio_stop(pi); pi = -1; } 
                        
                        std::cout << "[MIXR-1] System locked in Mode 3. Waiting for MATLAB to finish..." << std::endl;
                        mode3_notified = true;
                    }
                    
                    // Transmit the UI lock heartbeat
                    if (!network->send_packet(-2.0, -2.0)) break; 
                    usleep(1000000); 
                    continue;
                }

                if (mode3_notified) {
                    std::cout << "\n[MIXR-1] MATLAB teardown complete. Safely resuming backend." << std::endl;
                    mode3_notified = false;
                }

                // 3. Mode 2 Hardware Claim Initialization
                if (pi < 0) {
                    std::cout << "[MIXR-1] Claiming hardware for Mode 2..." << std::endl;
                    pi = pigpio_start(nullptr, nullptr);
                    if (pi >= 0) {
                        encoder = std::make_unique<PololuEncoder>(pi, 23, 24);
                        motor = std::make_unique<MotorController>(pi);
                        last_count = encoder->get_count(); 
                        filtered_rpm = 0.0; 
                        
                        // Reset the clock tracker exactly when hardware is claimed to prevent delta_sec spikes
                        last_time = std::chrono::high_resolution_clock::now();
                    } else {
                        std::cerr << "CRITICAL: pigpiod not ready. Retrying..." << std::endl;
                        usleep(2000000);
                        continue;
                    }
                }

                // 4. Bi-Directional Client Updates
                if (network->receive_command(current_pwm)) {
                    if (motor != nullptr) motor->set_pwm(current_pwm);
                    std::cout << "[MIXR-1] CMD Received -> Hardware PWM set to: " << current_pwm << std::endl;
                }

                // ==========================================
                // 5. INDUSTRY STANDARD EXACT-TIME METRICS & DSP
                // ==========================================

                /*
                 * ASYNCHRONOUS TIME-DELTA CAPTURE
                 * Linux is a General Purpose OS (GPOS), not an RTOS. The kernel scheduler 
                 * introduces unpredictable micro-stutters (jitter), causing loop times to vary.
                 * Capturing the exact nanosecond via hardware chronometry rather than assuming 
                 * a rigid 10ms loop time ensures the velocity division scales perfectly, 
                 * neutralizing OS-induced mathematical artifacts.
                 */
                auto current_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> exact_delta_sec = current_time - last_time;
                last_time = current_time;

                /*
                 * DISCRETE POSITION DIFFERENTIAL (Hybrid Method)
                 * Calculates the exact integer step change in quadrature states since the last loop.
                 * This cleanly decouples the high-speed hardware interrupts from the math loop.
                 */
                long long current_count = encoder->get_count();
                long long delta_ticks = current_count - last_count;
                last_count = current_count;
                
                double dt = exact_delta_sec.count();
                double exact_rpm = 0.0;
                
                if (dt > 0.0) {
                    /*
                     * MECHANICAL DEADBAND (NOISE GATE)
                     * When PWM is 0, the impeller is at rest. However, ambient workbench vibrations 
                     * or internal mechanical settling can cause the encoder disc to dither rapidly 
                     * between two magnetic poles (+1, -1). At a 100Hz sample rate, this 1-tick 
                     * dither amplifies into a persistent +/- 9.6 RPM false reading. This condition 
                     * clamps the output to true zero, preventing integral windup in subsequent controllers.
                     */
                    if (current_pwm == 0 && std::abs(delta_ticks) <= 2) {
                        exact_rpm = 0.0;
                    } else {
                        /*
                         * DIMENSIONAL DERIVATION
                         * 1. (delta_ticks / ENCODER_CPR) = Fractional shaft revolutions.
                         * 2. (60.0 / dt) = Conversion from 'revolutions per dt' to 'revolutions per minute'.
                         */
                        exact_rpm = (static_cast<double>(delta_ticks) / ENCODER_CPR) * (60.0 / dt);
                    }
                }
                
                /*
                 * INFINITE IMPULSE RESPONSE (IIR) LOW-PASS FILTER
                 * Quadrature encoders are discrete and truncate fractional movement, producing 
                 * 'Quantization Noise' (e.g., alternating between reading 10 and 11 ticks). 
                 * Passing raw noise to a PID Derivative term causes violent control instability.
                 * * Equation: y[n] = (alpha * x[n]) + ((1 - alpha) * y[n-1])
                 * * This Exponential Moving Average (EMA) applies a smoothing coefficient (RPM_ALPHA = 0.15).
                 * At 100Hz, this establishes a cutoff frequency (fc) of ~2.4 Hz. It aggressively 
                 * attenuates 100Hz high-frequency sensor error while preserving the true low-frequency 
                 * fluid mechanics and kinetic momentum of the physical motor.
                 */
                filtered_rpm = (RPM_ALPHA * exact_rpm) + ((1.0 - RPM_ALPHA) * filtered_rpm);
                
                // 6. Decoupled 10Hz Telemetry Transmission
                if (++telemetry_prescaler >= 10) {
                    telemetry_prescaler = 0;
                    if (!network->send_packet(filtered_rpm, 0.0)) {
                        std::cout << "\n[MIXR-1] Dashboard Disconnected." << std::endl;
                        break; 
                    }
                }
                
                // Base loop delay (~100Hz pacing constraint)
                usleep(LOOP_DELAY_US); 
            }

            // Safe shutdown cleanup on loop break
            motor.reset();
            encoder.reset();
            if (pi >= 0) pigpio_stop(pi);
        }
        network->stop_server();
    }

    std::cout << "\n[MIXR-1] Intercepted stop signal. Daemon safely offline." << std::endl;
    return 0;
}
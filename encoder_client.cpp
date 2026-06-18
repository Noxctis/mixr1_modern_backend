#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstring>
#include <memory>
#include <chrono> // Added for high-resolution industry-standard timing
#include <sys/socket.h>
#include <netinet/in.h>

// --- Global Configuration ---
constexpr double POLOLU_CPR = 617.35;
constexpr int TCP_PORT = 5000;
// 10,000 microseconds = 10ms = 100Hz Hardware Control Loop
constexpr int LOOP_DELAY_US = 10000; 

std::atomic<bool> run_loop(true);

void signal_handler(int signum) {
    run_loop = false;
}

// ==========================================
// SYSTEM MODULE: PROCESS MONITOR
// ==========================================
class ProcessMonitor {
public:
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
    volatile uint8_t state = 0;
    const int QUAD_STATES[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    static void isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
        static_cast<PololuEncoder*>(user)->decode_state(gpio, level);
    }

    void decode_state(unsigned gpio, unsigned level) {
        uint8_t val_a = (gpio == pin_a) ? level : gpio_read(pi_handle, pin_a);
        uint8_t val_b = (gpio == pin_b) ? level : gpio_read(pi_handle, pin_b);

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
// HARDWARE MODULE: MOTOR CONTROLLER
// ==========================================
class MotorController {
private:
    int pi_handle;
    const unsigned int M1EN = 15;
    const unsigned int M1INA = 17;
    const unsigned int M1PWM = 18;
    const unsigned int M1INB = 27;

public:
    MotorController(int pi) : pi_handle(pi) {
        set_mode(pi_handle, M1EN, PI_OUTPUT);
        set_mode(pi_handle, M1INA, PI_OUTPUT);
        set_mode(pi_handle, M1PWM, PI_OUTPUT);
        set_mode(pi_handle, M1INB, PI_OUTPUT);

        set_PWM_frequency(pi_handle, M1PWM, 20000); 
        set_PWM_range(pi_handle, M1PWM, 255); 

        gpio_write(pi_handle, M1EN, 1);
        gpio_write(pi_handle, M1INA, 1);
        gpio_write(pi_handle, M1INB, 0);
        set_PWM_dutycycle(pi_handle, M1PWM, 0); 
    }

    ~MotorController() {
        stop_motor();
    }

    void set_pwm(int duty_cycle) {
        if (duty_cycle < 0) duty_cycle = 0;
        if (duty_cycle > 255) duty_cycle = 255;
        set_PWM_dutycycle(pi_handle, M1PWM, duty_cycle);
    }

    void stop_motor() {
        set_PWM_dutycycle(pi_handle, M1PWM, 0);
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

            // --- FILTER VARIABLES DISABLED ---
            // double filtered_rpm = 0.0;
            // const double RPM_ALPHA = 0.15; 

            // High-resolution clock tracker for exact RPM math
            auto last_time = std::chrono::high_resolution_clock::now();

            while (run_loop) {
                if (++simulink_check_counter >= 100) {
                    simulink_check_counter = 0;
                    simulink_is_active = ProcessMonitor::is_simulink_running();
                }

                if (simulink_is_active) {
                    if (!mode3_notified) {
                        std::cout << "\n[MIXR-1] MATLAB detected. Releasing hardware pins..." << std::endl;
                        motor.reset();
                        encoder.reset();
                        if (pi >= 0) { pigpio_stop(pi); pi = -1; }
                        
                        std::cout << "[MIXR-1] System locked in Mode 3. Waiting for MATLAB to finish..." << std::endl;
                        mode3_notified = true;
                    }
                    
                    if (!network->send_packet(-2.0, -2.0)) break; 
                    usleep(1000000); 
                    continue;
                }

                if (mode3_notified) {
                    std::cout << "\n[MIXR-1] MATLAB teardown complete. Safely resuming backend." << std::endl;
                    mode3_notified = false;
                }

                if (pi < 0) {
                    std::cout << "[MIXR-1] Claiming hardware for Mode 2..." << std::endl;
                    pi = pigpio_start(nullptr, nullptr);
                    if (pi >= 0) {
                        encoder = std::make_unique<PololuEncoder>(pi, 23, 24);
                        motor = std::make_unique<MotorController>(pi);
                        last_count = encoder->get_count(); 
                        
                        // Reset the clock tracker on hardware claim
                        last_time = std::chrono::high_resolution_clock::now();
                    } else {
                        std::cerr << "CRITICAL: pigpiod not ready. Retrying..." << std::endl;
                        usleep(2000000);
                        continue;
                    }
                }

                // 1. Process Real-Time Commands
                if (network->receive_command(current_pwm)) {
                    if (motor != nullptr) motor->set_pwm(current_pwm);
                    std::cout << "[MIXR-1] CMD Received -> Hardware PWM set to: " << current_pwm << std::endl;
                }

                // 2. 100Hz Physical State Math (INDUSTRY STANDARD EXACT-TIME METHOD)
                // We capture the exact nanosecond we read the count. By dividing the 
                // raw delta count by the exact delta time in seconds, we completely eliminate 
                // RPM spikes caused by Linux OS scheduling jitter.
                
                auto current_time = std::chrono::high_resolution_clock::now();
                std::chrono::duration<double> exact_delta_sec = current_time - last_time;
                last_time = current_time;

                long long current_count = encoder->get_count();
                long long delta_ticks = current_count - last_count;
                last_count = current_count;
                
                double dt = exact_delta_sec.count();
                double exact_rpm = 0.0;
                
                if (dt > 0.0) {
                    // exact_rpm = (Revolutions) / (Minutes)
                    // exact_rpm = (delta_ticks / POLOLU_CPR) / (dt_seconds / 60.0)
                    exact_rpm = (static_cast<double>(delta_ticks) / POLOLU_CPR) * (60.0 / dt);
                }

                // --- FILTER DISABLED ---
                // filtered_rpm = (RPM_ALPHA * exact_rpm) + ((1.0 - RPM_ALPHA) * filtered_rpm);
                
                // 3. Decoupled 10Hz Telemetry Transmission
                if (++telemetry_prescaler >= 10) {
                    telemetry_prescaler = 0;
                    // Transmitting exact_rpm directly instead of filtered_rpm
                    if (!network->send_packet(exact_rpm, 0.0)) {
                        std::cout << "\n[MIXR-1] Dashboard Disconnected." << std::endl;
                        break; 
                    }
                }
                
                // Paces the hardware control loop to ~100Hz. The Exact-Time math above
                // guarantees accurate RPM regardless of microsecond drift in this usleep.
                usleep(LOOP_DELAY_US); 
            }

            motor.reset();
            encoder.reset();
            if (pi >= 0) pigpio_stop(pi);
        }
        network->stop_server();
    }

    std::cout << "\n[MIXR-1] Intercepted stop signal. Daemon safely offline." << std::endl;
    return 0;
}
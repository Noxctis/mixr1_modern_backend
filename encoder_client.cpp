#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstring>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>

// --- Global Configuration ---
constexpr double POLOLU_CPR = 617.35;
constexpr int TCP_PORT = 5000;
constexpr int LOOP_DELAY_US = 100000; // 10Hz telemetry rate

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
        // [r] bypasses the shell mirror bug.
        // raspberrypi_get matches the 15-character strict kernel truncation limit.
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
    const unsigned int M1PWM = 18; // Hardware PWM
    const unsigned int M1INB = 27;

public:
    MotorController(int pi) : pi_handle(pi) {
        set_mode(pi_handle, M1EN, PI_OUTPUT);
        set_mode(pi_handle, M1INA, PI_OUTPUT);
        set_mode(pi_handle, M1PWM, PI_OUTPUT);
        set_mode(pi_handle, M1INB, PI_OUTPUT);

        // 20kHz to eliminate motor coil whine
        set_PWM_frequency(pi_handle, M1PWM, 20000); 
        set_PWM_range(pi_handle, M1PWM, 255); 

        // Default Forward
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
        gpio_write(pi_handle, M1EN, 0); // Drop standby line to cut power
    }
};

// ==========================================
// NETWORK MODULE: TCP SERVER
// ==========================================
class TelemetryServer {
private:
    int server_fd = -1;
    int client_socket = -1;

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

    // NON-BLOCKING READ: Grabs inbound UI commands without slowing down telemetry
    bool receive_command(int& new_pwm) {
        if (client_socket < 0) return false;
        char buffer[128];
        ssize_t bytes = recv(client_socket, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
        
        if (bytes > 0) {
            buffer[bytes] = '\0';
            std::string msg(buffer);
            size_t pos = msg.find("CMD:PWM,");
            if (pos != std::string::npos) {
                try {
                    new_pwm = std::stoi(msg.substr(pos + 8));
                    return true;
                } catch (...) {}
            }
        }
        return false;
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
            int simulink_check_counter = 10;
            bool simulink_is_active = false;
            long long last_count = 0;
            int current_pwm = 0;

            while (run_loop) {
                if (++simulink_check_counter >= 10) {
                    simulink_check_counter = 0;
                    simulink_is_active = ProcessMonitor::is_simulink_running();
                }

                if (simulink_is_active) {
                    if (!mode3_notified) {
                        std::cout << "\n[MIXR-1] MATLAB detected. Releasing hardware pins..." << std::endl;
                        // Destructors safely shutdown PWM and GPIO before releasing the kernel
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

                // --- Mode 2 Hardware Logic ---
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
                    } else {
                        std::cerr << "CRITICAL: pigpiod not ready. Retrying..." << std::endl;
                        usleep(2000000);
                        continue;
                    }
                }

                // 1. Process incoming UI Commands
                if (network->receive_command(current_pwm)) {
                    if (motor != nullptr) motor->set_pwm(current_pwm);
                    std::cout << "[MIXR-1] UI Command Received -> Setting PWM to: " << current_pwm << std::endl;
                }

                // 2. Read Physical State
                long long current_count = encoder->get_count();
                long long delta = current_count - last_count;
                last_count = current_count;
                double rpm = (delta * 600.0) / POLOLU_CPR;
                
                // 3. Transmit
                if (!network->send_packet(rpm, 0.0)) {
                    std::cout << "\n[MIXR-1] Dashboard Disconnected." << std::endl;
                    break; 
                }
                
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
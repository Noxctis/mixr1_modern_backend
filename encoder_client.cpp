#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstring>
#include <memory>
#include <sys/socket.h>
#include <netinet/in.h>
#include <sys/select.h>

// --- Global Configuration ---
constexpr double POLOLU_CPR = 617.35;
constexpr int TCP_PORT = 5000;
constexpr int LOOP_DELAY_US = 100000; // 100ms hardware sync

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
        // The bracket [r] prevents the system shell from detecting its own search command
        const char* cmd = "pgrep -f \"[r]aspberrypi_gettingstarted\" > /dev/null 2>&1";
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
// NETWORK MODULE: TCP TELEMETRY SERVER
// ==========================================
class TelemetryServer {
private:
    int server_fd = -1;
    int client_socket = -1;

public:
    ~TelemetryServer() {
        stop_server();
    }

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
            
            struct timeval tv{1, 0}; // 1 second timeout to allow Ctrl+C checks

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

    // 1. Initialize the network object outside the hardware loop
    auto network = std::make_unique<TelemetryServer>();

    while (run_loop) {
        // 2. ALWAYS open the port first so the dashboard can connect
        if (!network->start_server(TCP_PORT)) {
            std::cerr << "CRITICAL: Port " << TCP_PORT << " locked. Retrying..." << std::endl;
            usleep(2000000);
            continue;
        }

        // 3. Wait for the Python UI to connect
        if (network->wait_for_client()) {
            std::cout << "[MIXR-1] Dashboard Connected. Managing state engine..." << std::endl;
            
            int pi = -1;
            std::unique_ptr<PololuEncoder> motor = nullptr;

            // 4. The active data loop
            while (run_loop) {
                // Check Simulink status dynamically while connected
                if (ProcessMonitor::is_simulink_running()) {
                    
                    // If we had the hardware, let it go so MATLAB can use it
                    if (motor != nullptr) {
                        std::cout << "\n[MIXR-1] MATLAB detected. Releasing hardware pins..." << std::endl;
                        motor.reset();
                        if (pi >= 0) { pigpio_stop(pi); pi = -1; }
                    }
                    
                    std::cout << "[MIXR-1] MATLAB Simulink Active (Mode 3). Streaming state lock..." << std::endl;
                    
                    // Send the special Mode 3 packet to the dashboard
                    if (!network->send_packet(-2.0, -2.0)) {
                        break; // Break if dashboard closes
                    }
                    usleep(1000000); // Wait 1 second before checking again
                    continue;
                }

                // --- Mode 2 Hardware Logic ---
                if (motor == nullptr) {
                    std::cout << "\n[MIXR-1] Claiming hardware for Mode 2..." << std::endl;
                    pi = pigpio_start(nullptr, nullptr);
                    if (pi >= 0) {
                        motor = std::make_unique<PololuEncoder>(pi, 23, 24);
                    } else {
                        std::cerr << "CRITICAL: pigpiod not ready. Retrying..." << std::endl;
                        usleep(2000000);
                        continue;
                    }
                }

                long long last_count = motor->get_count();
                usleep(LOOP_DELAY_US);
                long long current_count = motor->get_count();
                
                double rpm = ((current_count - last_count) * 600.0) / POLOLU_CPR;
                
                if (!network->send_packet(rpm, 0.0)) {
                    std::cout << "\n[MIXR-1] Dashboard Disconnected." << std::endl;
                    break; 
                }
            }

            // Cleanup hardware when dashboard disconnects
            if (motor != nullptr) motor.reset();
            if (pi >= 0) pigpio_stop(pi);
        }
        
        // Shut down the server socket before the next connection attempt
        network->stop_server();
    }

    std::cout << "\n[MIXR-1] Intercepted stop signal. Daemon safely offline." << std::endl;
    return 0;
}
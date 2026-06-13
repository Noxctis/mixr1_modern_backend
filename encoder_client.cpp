#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <csignal>
#include <atomic>
#include <cstring>
#include <sys/socket.h>
#include <netinet/in.h>

std::atomic<bool> run_loop(true);

void signal_handler(int signum) {
    run_loop = false;
}

// ==========================================
// SYSTEM CHECK: MATLAB PROCESS DETECTOR
// ==========================================
bool is_simulink_running() {
    // Replace "water_level_model" with your exact Simulink deployment name
    const char* cmd = "pgrep -x \"water_level_model\" > /dev/null 2>&1";
    int result = system(cmd);
    return (result == 0);
}

// ==========================================
// HARDWARE MODULE: POLOLU ENCODER
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
        PololuEncoder* enc = static_cast<PololuEncoder*>(user);
        enc->decode_state(gpio, level);
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
    bool start_server(int port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) return false;
        if (listen(server_fd, 1) < 0) return false;
        return true;
    }

    bool wait_for_client() {
        std::cout << "[MIXR-1] Mode 2 Active. Waiting for Dashboard (Port 5000)..." << std::endl;
        
        while (run_loop) {
            fd_set readfds;
            FD_ZERO(&readfds);
            FD_SET(server_fd, &readfds);
            
            struct timeval tv;
            tv.tv_sec = 1;  // Wake up every 1 second
            tv.tv_usec = 0;

            // Check if there is an incoming connection on the port
            int activity = select(server_fd + 1, &readfds, NULL, NULL, &tv);

            if (activity > 0 && FD_ISSET(server_fd, &readfds)) {
                // Client found, accept the connection
                client_socket = accept(server_fd, nullptr, nullptr);
                return (client_socket >= 0);
            }
            
            // If activity == 0, the 1-second timer expired. 
            // The while loop restarts, checking if Ctrl+C (run_loop) was pressed.
        }
        
        // If run_loop becomes false (Ctrl+C pressed), safely break out
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
// MAIN AUTO-RECOVERY LOOP
// ==========================================
int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGPIPE, SIG_IGN); 

    // Calibration for 50:1 Micro Metal Gearmotor with 12 CPR Encoder
    const double POLOLU_CPR = 617.35; 

    while (run_loop) {
        
        // --- STATE 1: CHECK FOR MATLAB (MODE 3) ---
        if (is_simulink_running()) {
            std::cout << "[MIXR-1] MATLAB Simulink Active (Mode 3). Backend sleeping..." << std::endl;
            usleep(2000000); 
            continue; 
        }

        // --- STATE 2: INITIALIZE MODE 2 HARDWARE ---
        std::cout << "\n[MIXR-1] MATLAB offline. Claiming hardware for Mode 2..." << std::endl;
        int pi = pigpio_start(NULL, NULL);
        if (pi < 0) {
            std::cerr << "CRITICAL: pigpio daemon not found. Retrying..." << std::endl;
            usleep(2000000);
            continue;
        }

        PololuEncoder* motor = new PololuEncoder(pi, 23, 24);
        TelemetryServer* network = new TelemetryServer();
        
        if (!network->start_server(5000)) {
            std::cerr << "CRITICAL: Port 5000 locked. Retrying..." << std::endl;
            delete motor; 
            delete network; 
            pigpio_stop(pi);
            usleep(2000000);
            continue;
        }

        long long last_count = motor->get_count();
        int simulink_check_counter = 0;

        // --- STATE 3: ACTIVE STREAMING ---
        if (network->wait_for_client()) {
            std::cout << "[MIXR-1] Dashboard Connected. Streaming..." << std::endl;
            
            while (run_loop) {
                // Check if MATLAB just started mid-run (Every 2 seconds)
                simulink_check_counter++;
                if (simulink_check_counter >= 20) { 
                    simulink_check_counter = 0;
                    if (is_simulink_running()) {
                        std::cout << "\n[MIXR-1] ALERT: MATLAB detected mid-run! Yielding control." << std::endl;
                        network->send_packet(-1.0, -1.0); // Send Kill Code to Dashboard
                        break; 
                    }
                }

                // Execute Math
                long long current_count = motor->get_count();
                long long delta = current_count - last_count;
                last_count = current_count;
                
                double rpm = (delta * 600.0) / POLOLU_CPR;
                double dummy_torque = 0.0;
                
                // Transmit dual-variable string
                if (!network->send_packet(rpm, dummy_torque)) {
                    std::cout << "\n[MIXR-1] Dashboard Disconnected." << std::endl;
                    break; 
                }

                usleep(100000); // 100ms hardware sync window
            }
        }

        // --- STATE 4: CLEAN DISMANTLE (Preparing for Mode 3) ---
        network->stop_server();
        delete motor;
        delete network;
        pigpio_stop(pi);
        std::cout << "[MIXR-1] Hardware released." << std::endl;
        
        // Loop returns to State 1
    }

    std::cout << "\n[MIXR-1] Intercepted stop signal. Backend safely offline." << std::endl;
    return 0;
}
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
// HARDWARE MODULE: POLOLU QUADRATURE ENCODER
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
    int server_fd;
    int client_socket = -1;
public:
    TelemetryServer(int port) {
        server_fd = socket(AF_INET, SOCK_STREAM, 0);
        int opt = 1;
        setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

        sockaddr_in address;
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = INADDR_ANY;
        address.sin_port = htons(port);

        bind(server_fd, (struct sockaddr*)&address, sizeof(address));
        listen(server_fd, 1);
    }

    ~TelemetryServer() {
        if (client_socket >= 0) close(client_socket);
        close(server_fd);
    }

    bool wait_for_client() {
        std::cout << "[MIXR-1] Pololu Validation Active. Waiting for GUI (Port 5000)..." << std::endl;
        client_socket = accept(server_fd, nullptr, nullptr);
        return (client_socket >= 0);
    }

    bool send_packet(double rpm, double raw_delta) {
        if (client_socket < 0) return false;
        std::string packet = std::to_string(rpm) + "," + std::to_string(raw_delta) + "\n";
        ssize_t sent = send(client_socket, packet.c_str(), packet.length(), MSG_NOSIGNAL);
        return (sent > 0);
    }

    void disconnect_client() {
        if (client_socket >= 0) close(client_socket);
        client_socket = -1;
    }
};

// ==========================================
// MAIN EXECUTION LOOP
// ==========================================
int main() {
    std::signal(SIGINT, signal_handler);
    std::signal(SIGPIPE, SIG_IGN); 

    int pi = pigpio_start(NULL, NULL); 
    if (pi < 0) return 1;

    PololuEncoder pololu_motor(pi, 23, 24);
    TelemetryServer network(5000);

    // EXACT Calibration for 50:1 Micro Metal Gearmotor with 12 CPR Encoder
    const double POLOLU_CPR = 617.35; 
    long long last_count = 0;

    while (run_loop) {
        if (network.wait_for_client()) {
            std::cout << "[MIXR-1] Dashboard Connected. Streaming Telemetry..." << std::endl;
            
            while (run_loop) {
                long long current_count = pololu_motor.get_count();
                
                long long delta = current_count - last_count;
                last_count = current_count;
                
                // Convert raw 100ms pulse delta into true RPM
                double rpm = (delta * 600.0) / POLOLU_CPR;
                
                // Transmit True RPM and a placeholder Torque (0.0) for the thesis graphs
                double dummy_torque = 0.0; 

                if (!network.send_packet(rpm, dummy_torque)) {
                    std::cout << "\n[MIXR-1] Dashboard Disconnected." << std::endl;
                    network.disconnect_client();
                    break; 
                }

                usleep(100000); // 100ms hardware sync window
            }
        }
    }

    pigpio_stop(pi);
    std::cout << "[MIXR-1] Hardware lines detached. Core backend safely offline." << std::endl;
    return 0;
}
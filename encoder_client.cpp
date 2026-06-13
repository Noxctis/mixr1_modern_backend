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

// --- HARDWARE CLASS: Encapsulates all encoder logic ---
class PololuEncoder {
private:
    int pi_handle;
    unsigned int pin_a, pin_b; // FIX: Changed from int to unsigned int
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
    // FIX: Changed constructor parameters 'a' and 'b' to unsigned int
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

// --- MAIN EXECUTION THREAD ---
int main() {
    // 1. Prevent network disconnections from crashing the Linux process
    std::signal(SIGINT, signal_handler);
    std::signal(SIGPIPE, SIG_IGN); 

    int pi = pigpio_start(NULL, NULL); 
    if (pi < 0) return 1;

    PololuEncoder mixer_encoder(pi, 23, 24);

    // 2. Configure TCP Server
    int server_fd = socket(AF_INET, SOCK_STREAM, 0);
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address;
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(5000);

    bind(server_fd, (struct sockaddr*)&address, sizeof(address));
    listen(server_fd, 1);

    long long last_count = 0;

    // 3. Resilient Server Loop
    while (run_loop) {
        std::cout << "[MIXR-1] Backend active. Waiting for GUI connection..." << std::endl;
        int client_socket = accept(server_fd, nullptr, nullptr);
        
        if (client_socket >= 0) {
            std::cout << "[MIXR-1] UI Connected. Streaming telemetry..." << std::endl;
            
            while (run_loop) {
                long long current = mixer_encoder.get_count();
                long long delta = current - last_count;
                last_count = current;

                std::string packet = std::to_string(current) + "," + std::to_string(delta) + "\n";
                
                // MSG_NOSIGNAL prevents crash if Python socket drops abruptly
                ssize_t bytes_sent = send(client_socket, packet.c_str(), packet.length(), MSG_NOSIGNAL);
                
                if (bytes_sent < 0) {
                    std::cout << "\n[MIXR-1] UI Disconnected. Halting stream." << std::endl;
                    break; // Break inner loop to go back to 'accept()' waiting
                }

                usleep(100000); // 100ms hardware sync window
            }
            close(client_socket);
        }
    }

    close(server_fd);
    pigpio_stop(pi);
    std::cout << "[MIXR-1] Hardware lines detached. Core backend safely offline." << std::endl;
    return 0;
}
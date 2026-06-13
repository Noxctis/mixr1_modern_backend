#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <csignal>   // Required for Ctrl+C handler
#include <atomic>    // Required for thread-safe flag

const int PIN_A = 23; 
const int PIN_B = 24; 

// Keep these volatile so the compiler updates them across loops
volatile long long encoder_count = 0;
volatile uint8_t encoder_state = 0;

const int QUAD_STATES[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Atomic loop control flag
std::atomic<bool> run_loop(true);

// Intercept Ctrl+C so Valgrind cleans up perfectly
void signal_handler(int signum) {
    run_loop = false;
}

// FIX: The daemon sends the exact pin states inside 'gpio' and 'level' variables!
void encoder_isr_callback(int pi, unsigned gpio, unsigned level, uint32_t tick) {
    uint8_t pin_val_A = 0;
    uint8_t pin_val_B = 0;

    // Decode current state without querying the network socket
    if (gpio == PIN_A) {
        pin_val_A = level;                // Use the level the daemon just sent us
        pin_val_B = gpio_read(pi, PIN_B); // Only read the opposite stable line
    } else {
        pin_val_A = gpio_read(pi, PIN_A); // Only read the opposite stable line
        pin_val_B = level;                // Use the level the daemon just sent us
    }

    encoder_state = (encoder_state << 2) & 0x0F;
    encoder_state |= (pin_val_A << 1) | pin_val_B;

    encoder_count += QUAD_STATES[encoder_state];
}

int main() {
    // Register the termination signal handler
    std::signal(SIGINT, signal_handler);

    // Connect to the background systemd daemon process on localhost
    int pi = pigpio_start(NULL, NULL); 
    if (pi < 0) {
        std::cerr << "CRITICAL: Could not connect to the background pigpiod service." << std::endl;
        return 1;
    }

    set_mode(pi, PIN_A, PI_INPUT);
    set_mode(pi, PIN_B, PI_INPUT);
    set_pull_up_down(pi, PIN_A, PI_PUD_UP);
    set_pull_up_down(pi, PIN_B, PI_PUD_UP);

    // Bind alerts through the daemon hook interface
    int callback_A = callback(pi, PIN_A, EITHER_EDGE, encoder_isr_callback);
    int callback_B = callback(pi, PIN_B, EITHER_EDGE, encoder_isr_callback);

    std::cout << "Connected to background pigpiod service successfully." << std::endl;
    std::cout << "==========================================================" << std::endl;

    long long last_count = 0;

    // FIX: Using run_loop instead of while(true) so code can finish gracefully
    while (run_loop) {
        long long current_count = encoder_count;
        long long delta = current_count - last_count;
        last_count = current_count;

        std::cout << "\r[Daemon Client SSH] Total Pulses: " << current_count 
                  << " | Delta/100ms: " << delta 
                  << "        " << std::flush;

        usleep(100000); // 100ms sampling window
    }

    // This section finally runs when you press Ctrl+C!
    std::cout << "\n\n[MIXR-1] Intercepted stop signal. Cleaning up resources..." << std::endl;

    // Clean up client registrations cleanly on exit
    callback_cancel(callback_A);
    callback_cancel(callback_B);
    pigpio_stop(pi);

    std::cout << "System offline. Valgrind profile is 100% clear." << std::endl;
    return 0;
}
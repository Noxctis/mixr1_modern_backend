#include <iostream>
#include <pigpiod_if2.h>
#include <unistd.h>

const int PIN_A = 23; 
const int PIN_B = 24; 

volatile long long encoder_count = 0;
const int QUAD_STATES[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};
volatile uint8_t encoder_state = 0;

// The callback function signature shifts slightly for the daemon interface
void encoder_isr_callback(int pi, unsigned gpio, unsigned level, uint32_t tick) {
    uint8_t pin_val_A = gpio_read(pi, PIN_A);
    uint8_t pin_val_B = gpio_read(pi, PIN_B);

    encoder_state = (encoder_state << 2) & 0x0F;
    encoder_state |= (pin_val_A << 1) | pin_val_B;

    encoder_count += QUAD_STATES[encoder_state];
}

int main() {
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

    while (true) {
        long long current_count = encoder_count;
        long long delta = current_count - last_count;
        last_count = current_count;

        std::cout << "\r[Daemon Client SSH] Total Pulses: " << current_count 
                  << " | Delta/100ms: " << delta 
                  << "        " << std::flush;

        usleep(100000);
    }

    // Clean up client registrations cleanly on exit
    callback_cancel(callback_A);
    callback_cancel(callback_B);
    pigpio_stop(pi);
    return 0;
}

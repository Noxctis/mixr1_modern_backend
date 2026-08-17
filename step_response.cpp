/**
 * step_response.cpp
 * Purpose: Captures a 10% PWM step response at 100Hz with SCHED_FIFO priority.
 */
#include <iostream>
#include <fstream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <atomic>

// --- Hardware Pins ---
constexpr unsigned int PIN_ENC_A = 24;
constexpr unsigned int PIN_ENC_B = 23;
constexpr unsigned int PIN_PWM = 13;
constexpr unsigned int PIN_EN = 15;
constexpr unsigned int PIN_INA = 17;
constexpr unsigned int PIN_INB = 27;

// --- System Configuration ---
// Encoder directly on output shaft: 48 PPR * 4 edges = 192 CPR
constexpr double ENCODER_CPR = 192.0; 
constexpr int PWM_FREQ = 20000;
constexpr int LOOP_DELAY_US = 10000; // 100Hz

std::atomic<long long> encoder_count{0};
static constexpr int QUAD_STATES[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

// Hardware Interrupt Routine for Quadrature Encoder
void isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
    if (level > 1) return;
    static uint8_t state = 0;
    uint8_t val_a = gpio_read(pi, PIN_ENC_A);
    uint8_t val_b = gpio_read(pi, PIN_ENC_B);
    state = (state << 2) & 0x0F;
    state |= (val_a << 1) | val_b;
    
    // Reverse direction multiplier (-1) if RPM reads negative when spinning forward
    encoder_count.fetch_add(QUAD_STATES[state] * -1, std::memory_order_relaxed); 
}

int main() {
    // Elevate thread to Real-Time Priority to eliminate OS jitter
    sched_param sch;
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &sch);
    sch.sched_priority = 90;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
        std::cerr << "[WARNING] Failed to set SCHED_FIFO. Must run with sudo.\n";
    }

    // Initialize pigpiod
    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        std::cerr << "[CRITICAL] Cannot connect to pigpiod.\n";
        return 1;
    }

    // Configure Encoder Pins
    set_mode(pi, PIN_ENC_A, PI_INPUT); set_pull_up_down(pi, PIN_ENC_A, PI_PUD_UP);
    set_mode(pi, PIN_ENC_B, PI_INPUT); set_pull_up_down(pi, PIN_ENC_B, PI_PUD_UP);
    callback(pi, PIN_ENC_A, EITHER_EDGE, isr_router);
    callback(pi, PIN_ENC_B, EITHER_EDGE, isr_router);

    // Configure Motor Driver Pins
    set_mode(pi, PIN_EN, PI_OUTPUT); gpio_write(pi, PIN_EN, 1);
    set_mode(pi, PIN_INA, PI_OUTPUT); gpio_write(pi, PIN_INA, 1);
    set_mode(pi, PIN_INB, PI_OUTPUT); gpio_write(pi, PIN_INB, 0);
    set_mode(pi, PIN_PWM, PI_ALT0);

    // Prepare CSV Logging
    std::ofstream log_file("step_response_data.csv");
    log_file << "Time_s,Raw_RPM\n";

    std::cout << "[SYSTEM] Establishing 1-second baseline (0% PWM)...\n";
    hardware_PWM(pi, PIN_PWM, PWM_FREQ, 0);
    
    auto absolute_start = std::chrono::steady_clock::now();
    auto next_wake = absolute_start;
    long long last_count = encoder_count.load();

    // 1-Second Baseline (Stationary)
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - absolute_start).count() < 1.0) {
        log_file << std::chrono::duration<double>(std::chrono::steady_clock::now() - absolute_start).count() << ",0.0\n";
        next_wake += std::chrono::microseconds(LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
    }

    std::cout << "[SYSTEM] Triggering 10% PWM Step for 3 seconds...\n";
    int step_pwm = 409; // 10% of 4095
    hardware_PWM(pi, PIN_PWM, PWM_FREQ, (step_pwm * 1000000) / 4095);

    // 3-Second Step Response (Dynamic)
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - absolute_start;
        if (elapsed.count() > 4.0) break; // 1s baseline + 3s step test

        long long current_count = encoder_count.load();
        long long delta_ticks = current_count - last_count;
        last_count = current_count;

        // RPM = (ticks / CPR) * (60s / 0.01s)
        double rpm = (static_cast<double>(delta_ticks) / ENCODER_CPR) * 6000.0;
        
        log_file << elapsed.count() << "," << rpm << "\n";

        next_wake += std::chrono::microseconds(LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
    }

    // Shutdown hardware
    hardware_PWM(pi, PIN_PWM, PWM_FREQ, 0);
    gpio_write(pi, PIN_EN, 0);
    log_file.close();
    pigpio_stop(pi);

    std::cout << "[SYSTEM] Done. Data saved to 'step_response_data.csv'\n";
    return 0;
}
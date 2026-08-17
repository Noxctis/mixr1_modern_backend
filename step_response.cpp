/**
 * step_response.cpp
 * Purpose: Captures a 10% PWM step response at 100Hz with SCHED_FIFO priority using the AMT102-V.
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
constexpr unsigned int PIN_ENC_X = 22; // Index Pulse Pin
constexpr unsigned int PIN_PWM = 13;
constexpr unsigned int PIN_EN = 15;
constexpr unsigned int PIN_INA = 17;
constexpr unsigned int PIN_INB = 27;

// --- System Configuration ---
constexpr double ENCODER_CPR = 192.0; // 48 PPR * 4 edges
constexpr int PWM_FREQ = 20000;
constexpr int LOOP_DELAY_US = 10000; // 100Hz

// ==========================================
// AMT102 ENCODER MODULE
// ==========================================
class AMT102Encoder {
private:
    int pi_handle;
    unsigned int pin_a, pin_b, pin_x;
    int cb_a, cb_b, cb_x;
    
    std::atomic<long long> count{0};
    std::atomic<long long> revolutions{0};
    
    uint8_t state = 0;
    uint8_t val_a = 0;
    uint8_t val_b = 0;
    
    static constexpr int QUAD_STATES[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    static void isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
        if (level > 1) return; 
        static_cast<AMT102Encoder*>(user)->update_state(gpio, level);
    }

    static void isr_index(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
        if (level == 1) { 
            static_cast<AMT102Encoder*>(user)->revolutions.fetch_add(1, std::memory_order_relaxed);
        }
    }

    void update_state(unsigned gpio, unsigned level) {
        if (gpio == pin_a) val_a = level;
        else if (gpio == pin_b) val_b = level;

        state = (state << 2) & 0x0F;
        state |= (val_a << 1) | val_b;
        
        count.fetch_add(QUAD_STATES[state], std::memory_order_relaxed);
    }

public:
    AMT102Encoder(int pi, unsigned int a, unsigned int b, unsigned int x) : pi_handle(pi), pin_a(a), pin_b(b), pin_x(x) {
        set_mode(pi_handle, pin_a, PI_INPUT);
        set_mode(pi_handle, pin_b, PI_INPUT);
        set_mode(pi_handle, pin_x, PI_INPUT);
        
        set_pull_up_down(pi_handle, pin_a, PI_PUD_UP);
        set_pull_up_down(pi_handle, pin_b, PI_PUD_UP);
        set_pull_up_down(pi_handle, pin_x, PI_PUD_OFF); 
        
        val_a = gpio_read(pi_handle, pin_a);
        val_b = gpio_read(pi_handle, pin_b);
        
        cb_a = callback_ex(pi_handle, pin_a, EITHER_EDGE, isr_router, this);
        cb_b = callback_ex(pi_handle, pin_b, EITHER_EDGE, isr_router, this);
        cb_x = callback_ex(pi_handle, pin_x, RISING_EDGE, isr_index, this);
    }

    ~AMT102Encoder() {
        callback_cancel(cb_a);
        callback_cancel(cb_b);
        callback_cancel(cb_x);
    }

    [[nodiscard]] long long get_count() const { return count.load(std::memory_order_relaxed); }
    [[nodiscard]] long long get_revolutions() const { return revolutions.load(std::memory_order_relaxed); }
};

// ==========================================
// MAIN EXECUTOR
// ==========================================
int main() {
    sched_param sch;
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &sch);
    sch.sched_priority = 90;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
        std::cerr << "[WARNING] Failed to set SCHED_FIFO. Must run with sudo.\n";
    }

    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) return 1;

    AMT102Encoder encoder(pi, PIN_ENC_A, PIN_ENC_B, PIN_ENC_X);

    set_mode(pi, PIN_EN, PI_OUTPUT); gpio_write(pi, PIN_EN, 1);
    set_mode(pi, PIN_INA, PI_OUTPUT); gpio_write(pi, PIN_INA, 1);
    set_mode(pi, PIN_INB, PI_OUTPUT); gpio_write(pi, PIN_INB, 0);
    set_mode(pi, PIN_PWM, PI_ALT0);

    std::ofstream log_file("step_response_data.csv");
    log_file << "Time_s,Raw_RPM,Revolutions\n";

    hardware_PWM(pi, PIN_PWM, PWM_FREQ, 0);
    
    auto absolute_start = std::chrono::steady_clock::now();
    auto next_wake = absolute_start;
    long long last_count = encoder.get_count();

    // 1-Second Baseline
    while (std::chrono::duration<double>(std::chrono::steady_clock::now() - absolute_start).count() < 1.0) {
        log_file << std::chrono::duration<double>(std::chrono::steady_clock::now() - absolute_start).count() << ",0.0,0\n";
        next_wake += std::chrono::microseconds(LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
    }

    std::cout << "[SYSTEM] Triggering 10% PWM Step for 3 seconds...\n";
    int step_pwm = 409; 
    hardware_PWM(pi, PIN_PWM, PWM_FREQ, (step_pwm * 1000000) / 4095);

    // 3-Second Step
    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - absolute_start;
        if (elapsed.count() > 4.0) break; 

        long long current_count = encoder.get_count();
        long long delta_ticks = current_count - last_count;
        last_count = current_count;

        double rpm = (static_cast<double>(delta_ticks) / ENCODER_CPR) * 6000.0;
        
        log_file << elapsed.count() << "," << rpm << "," << encoder.get_revolutions() << "\n";

        next_wake += std::chrono::microseconds(LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
    }

    hardware_PWM(pi, PIN_PWM, PWM_FREQ, 0);
    gpio_write(pi, PIN_EN, 0);
    log_file.close();
    pigpio_stop(pi);

    std::cout << "[SYSTEM] Done. Data saved to 'step_response_data.csv'\n";
    return 0;
}
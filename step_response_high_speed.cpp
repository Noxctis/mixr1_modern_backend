/**
 * step_response_bisection.cpp
 * Purpose: Interactive Step Response Logger for Bisection Mapping under fluid load.
 */
#include <iostream>
#include <fstream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <chrono>
#include <thread>
#include <pthread.h>
#include <atomic>
#include <string>

// --- Hardware Pins ---
constexpr unsigned int PIN_ENC_A = 24; // Ensure these match your actual wiring
constexpr unsigned int PIN_ENC_B = 23;
constexpr unsigned int PIN_ENC_X = 22; 
constexpr unsigned int PIN_PWM = 13;
constexpr unsigned int PIN_EN = 15;
constexpr unsigned int PIN_INA = 17;
constexpr unsigned int PIN_INB = 27;

// --- System Configuration ---
constexpr double ENCODER_CPR = 192.0; 
constexpr int PWM_FREQ = 20000;
constexpr int LOOP_DELAY_US = 10000; // strictly 100Hz

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
    std::cout << "======================================\n";
    std::cout << " MIXR-1 HIGH-SPEED STEP RESPONSE TEST \n";
    std::cout << "======================================\n";
    std::cout << "1. Test 1 (0% to 10% PWM) - Stiction/Low Speed\n";
    std::cout << "2. Test 2 (15% to 25% PWM) - Bisect Low\n";
    std::cout << "3. Test 3 (35% to 45% PWM) - Midpoint\n";
    std::cout << "4. Test 4 (55% to 65% PWM) - Bisect High\n";
    std::cout << "5. Test 5 (70% to 80% PWM) - Nominal Max\n";
    std::cout << "Select test (1-5): ";

    int choice;
    std::cin >> choice;

    std::cout << "Select controller mode (1=OpenLoop, 2=PI): ";
    int controller_choice;
    std::cin >> controller_choice;

    std::cout << "Enable FIFO scheduling? (1=yes, 0=no): ";
    int fifo_choice;
    std::cin >> fifo_choice;

    int base_pct = 0, step_pct = 0;
    if (choice == 1) { base_pct = 0; step_pct = 10; }
    else if (choice == 2) { base_pct = 15; step_pct = 25; }
    else if (choice == 3) { base_pct = 35; step_pct = 45; }
    else if (choice == 4) { base_pct = 55; step_pct = 65; }
    else if (choice == 5) { base_pct = 70; step_pct = 80; }
    else { std::cerr << "Invalid choice. Exiting.\n"; return 1; }

    const std::string controller_mode = (controller_choice == 2) ? "PI" : "OpenLoop";
    const bool fifo_enabled = (fifo_choice == 1);
    const std::string fifo_label = fifo_enabled ? "FIFO" : "NoFIFO";
    const std::string run_label = controller_mode + "_" + fifo_label;

    int baseline_pwm = (base_pct * 4095) / 100;
    int step_pwm = (step_pct * 4095) / 100;
    std::string filename = "step_response_high_speed_" + run_label + "_" + std::to_string(base_pct) + "_" + std::to_string(step_pct) + ".csv";

    if (fifo_enabled) {
        sched_param sch;
        int policy;
        pthread_getschedparam(pthread_self(), &policy, &sch);
        sch.sched_priority = 90;
        if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
            std::cerr << "[WARNING] Failed to set SCHED_FIFO. Must run with sudo.\n";
        }
    }

    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) return 1;

    AMT102Encoder encoder(pi, PIN_ENC_A, PIN_ENC_B, PIN_ENC_X);

    set_mode(pi, PIN_EN, PI_OUTPUT); gpio_write(pi, PIN_EN, 1);
    set_mode(pi, PIN_INA, PI_OUTPUT); gpio_write(pi, PIN_INA, 1);
    set_mode(pi, PIN_INB, PI_OUTPUT); gpio_write(pi, PIN_INB, 0);
    set_mode(pi, PIN_PWM, PI_ALT0);

    std::ofstream log_file(filename);
    log_file << "Run_Label," << run_label << "\n";
    log_file << "Controller_Mode," << controller_mode << "\n";
    log_file << "FIFO_Enabled," << (fifo_enabled ? 1 : 0) << "\n";
    log_file << "Baseline_PWM_pct," << base_pct << "\n";
    log_file << "Step_PWM_pct," << step_pct << "\n";
    log_file << "Time_s,LoopPeriod_ms,Raw_RPM,Filtered_RPM,Target_RPM,Command_PWM,Revolutions,Controller_Mode,FIFO_Enabled,Baseline_PWM_pct,Step_PWM_pct,Run_Label\n";

    std::cout << "\n[SYSTEM] Spinning up to " << base_pct << "% PWM baseline...\n";
    hardware_PWM(pi, PIN_PWM, PWM_FREQ, (static_cast<long long>(baseline_pwm) * 1000000LL) / 4095LL);

    auto absolute_start = std::chrono::steady_clock::now();
    auto next_wake = absolute_start;
    auto last_loop_time = absolute_start;
    long long last_count = encoder.get_count();

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - absolute_start;
        if (elapsed.count() >= 1.0) break;

        auto loop_now = std::chrono::steady_clock::now();
        double loop_period_ms = std::chrono::duration<double, std::milli>(loop_now - last_loop_time).count();
        last_loop_time = loop_now;

        long long current_count = encoder.get_count();
        long long delta_ticks = current_count - last_count;
        last_count = current_count;

        double rpm = (static_cast<double>(delta_ticks) / ENCODER_CPR) * 6000.0 * -1.0;
        log_file << elapsed.count() << "," << loop_period_ms << "," << rpm << "," << rpm << ",0.0," << baseline_pwm << "," << encoder.get_revolutions() << "," << controller_mode << "," << (fifo_enabled ? 1 : 0) << "," << base_pct << "," << step_pct << "," << run_label << "\n";

        next_wake += std::chrono::microseconds(LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
    }

    std::cout << "[SYSTEM] Triggering " << step_pct << "% PWM Step for 3 seconds...\n";
    hardware_PWM(pi, PIN_PWM, PWM_FREQ, (static_cast<long long>(step_pwm) * 1000000LL) / 4095LL);

    while (true) {
        auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> elapsed = current_time - absolute_start;
        if (elapsed.count() > 4.0) break;

        auto loop_now = std::chrono::steady_clock::now();
        double loop_period_ms = std::chrono::duration<double, std::milli>(loop_now - last_loop_time).count();
        last_loop_time = loop_now;

        long long current_count = encoder.get_count();
        long long delta_ticks = current_count - last_count;
        last_count = current_count;

        double rpm = (static_cast<double>(delta_ticks) / ENCODER_CPR) * 6000.0 * -1.0;
        log_file << elapsed.count() << "," << loop_period_ms << "," << rpm << "," << rpm << ",0.0," << step_pwm << "," << encoder.get_revolutions() << "," << controller_mode << "," << (fifo_enabled ? 1 : 0) << "," << base_pct << "," << step_pct << "," << run_label << "\n";

        next_wake += std::chrono::microseconds(LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
    }

    hardware_PWM(pi, PIN_PWM, PWM_FREQ, 0);
    gpio_write(pi, PIN_EN, 0);
    log_file.close();
    pigpio_stop(pi);

    std::cout << "[SYSTEM] Done. Data saved to '" << filename << "'\n";
    return 0;
}
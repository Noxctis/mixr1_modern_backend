#pragma once
#include <atomic>
#include <cstdint>

// Interrupt-driven EC11 reader: pigpio callbacks decode every A/B edge as it happens,
// so no transition is lost between main-loop iterations.
class EC11Input {
private:
    int pi_handle;
    int pin_a;
    int pin_b;
    int pin_sw;
    int cb_a = -1, cb_b = -1, cb_sw = -1;

    // touched only from the pigpio callback thread
    int val_a = 0, val_b = 0, last_ab = 0;
    uint32_t last_press_tick = 0;

    // shared with the main loop
    std::atomic<int> delta_accum{0};
    std::atomic<bool> press_flag{false};

    static void isr_ab(int pi, unsigned gpio, unsigned level, uint32_t tick, void* user);
    static void isr_sw(int pi, unsigned gpio, unsigned level, uint32_t tick, void* user);
    void on_ab(unsigned gpio, unsigned level);
    void on_sw(unsigned level, uint32_t tick);

public:
    EC11Input(int pi, int gpio_a, int gpio_b, int gpio_sw);
    ~EC11Input();
    EC11Input(const EC11Input&) = delete;
    EC11Input& operator=(const EC11Input&) = delete;

    int read_delta();          // quadrature transitions since last call (signed)
    bool button_pressed();     // true once per press
};
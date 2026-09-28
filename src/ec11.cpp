#include "ec11.hpp"
#include "config.hpp"
#include <pigpiod_if2.h>

namespace {
// transition_table[previous AB][new AB] -> -1, 0, +1
const int TRANSITION_TABLE[4][4] = {
    {0, -1, 1, 0},
    {1, 0, 0, -1},
    {-1, 0, 0, 1},
    {0, 1, -1, 0}
};
}

void EC11Input::isr_ab(int, unsigned gpio, unsigned level, uint32_t, void* user) {
    if (level > 1) return;   // watchdog timeout
    static_cast<EC11Input*>(user)->on_ab(gpio, level);
}

void EC11Input::isr_sw(int, unsigned, unsigned level, uint32_t tick, void* user) {
    if (level > 1) return;
    static_cast<EC11Input*>(user)->on_sw(level, tick);
}

void EC11Input::on_ab(unsigned gpio, unsigned level) {
    if (static_cast<int>(gpio) == pin_a) val_a = static_cast<int>(level);
    else                                 val_b = static_cast<int>(level);

    const int ab = (val_a << 1) | val_b;
    if (ab == last_ab) return;
    const int delta = TRANSITION_TABLE[last_ab][ab];
    last_ab = ab;
    if (delta != 0) delta_accum.fetch_add(delta, std::memory_order_relaxed);
}

void EC11Input::on_sw(unsigned level, uint32_t tick) {
    if (level != 0) return;                                        // act on press (active low)
    if (last_press_tick != 0 && (tick - last_press_tick) < Config::EC11_BUTTON_DEBOUNCE_US) return;
    last_press_tick = tick;
    press_flag.store(true, std::memory_order_relaxed);
}

EC11Input::EC11Input(int pi, int gpio_a, int gpio_b, int gpio_sw)
    : pi_handle(pi), pin_a(gpio_a), pin_b(gpio_b), pin_sw(gpio_sw) {
    set_mode(pi_handle, pin_a, PI_INPUT);
    set_mode(pi_handle, pin_b, PI_INPUT);
    set_mode(pi_handle, pin_sw, PI_INPUT);
    set_pull_up_down(pi_handle, pin_a, PI_PUD_UP);
    set_pull_up_down(pi_handle, pin_b, PI_PUD_UP);
    set_pull_up_down(pi_handle, pin_sw, PI_PUD_UP);

    // filter contact bounce in pigpio (edges must be stable this long)
    set_glitch_filter(pi_handle, pin_a, Config::EC11_GLITCH_US);
    set_glitch_filter(pi_handle, pin_b, Config::EC11_GLITCH_US);

    val_a = gpio_read(pi_handle, pin_a) ? 1 : 0;
    val_b = gpio_read(pi_handle, pin_b) ? 1 : 0;
    last_ab = (val_a << 1) | val_b;

    cb_a = callback_ex(pi_handle, pin_a, EITHER_EDGE, isr_ab, this);
    cb_b = callback_ex(pi_handle, pin_b, EITHER_EDGE, isr_ab, this);
    cb_sw = callback_ex(pi_handle, pin_sw, EITHER_EDGE, isr_sw, this);
}

EC11Input::~EC11Input() {
    if (cb_a >= 0) callback_cancel(cb_a);
    if (cb_b >= 0) callback_cancel(cb_b);
    if (cb_sw >= 0) callback_cancel(cb_sw);
}

int EC11Input::read_delta() {
    return delta_accum.exchange(0, std::memory_order_relaxed);
}

bool EC11Input::button_pressed() {
    return press_flag.exchange(false, std::memory_order_relaxed);
}
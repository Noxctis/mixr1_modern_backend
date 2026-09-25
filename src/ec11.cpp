#include "ec11.hpp"
#include <pigpiod_if2.h>

EC11Input::EC11Input(int pi, int gpio_a, int gpio_b, int gpio_sw)
    : pi_handle(pi), pin_a(gpio_a), pin_b(gpio_b), pin_sw(gpio_sw) {
    set_mode(pi_handle, pin_a, PI_INPUT);
    set_mode(pi_handle, pin_b, PI_INPUT);
    set_mode(pi_handle, pin_sw, PI_INPUT);
    set_pull_up_down(pi_handle, pin_a, PI_PUD_UP);
    set_pull_up_down(pi_handle, pin_b, PI_PUD_UP);
    set_pull_up_down(pi_handle, pin_sw, PI_PUD_UP);

    const int a = gpio_read(pi_handle, pin_a) ? 1 : 0;
    const int b = gpio_read(pi_handle, pin_b) ? 1 : 0;
    last_ab = (a << 1) | b;
    initialized = true;
}

int EC11Input::read_delta() {
    if (!initialized) return 0;
    const int a = gpio_read(pi_handle, pin_a) ? 1 : 0;
    const int b = gpio_read(pi_handle, pin_b) ? 1 : 0;
    const int ab = (a << 1) | b;
    if (ab == last_ab) return 0;

    static const int transition_table[4][4] = {
        {0, -1, 1, 0},
        {1, 0, 0, -1},
        {-1, 0, 0, 1},
        {0, 1, -1, 0}
    };
    const int delta = transition_table[last_ab][ab];
    last_ab = ab;
    return delta;
}

bool EC11Input::button_pressed() {
    if (!initialized) return false;
    const bool pressed_now = gpio_read(pi_handle, pin_sw) == 0;
    const bool edge_pressed = pressed_now && !last_button_pressed;
    last_button_pressed = pressed_now;
    return edge_pressed;
}

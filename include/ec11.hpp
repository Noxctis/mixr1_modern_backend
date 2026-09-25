#pragma once

class EC11Input {
private:
    int pi_handle;
    int pin_a;
    int pin_b;
    int pin_sw;
    int last_ab = 0;
    bool initialized = false;
    bool last_button_pressed = false;

public:
    EC11Input(int pi, int gpio_a, int gpio_b, int gpio_sw);
    int read_delta();
    bool button_pressed();
};

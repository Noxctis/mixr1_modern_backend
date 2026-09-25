#pragma once

class LocalControl {
private:
    int pi_handle;
    unsigned int pin_a;
    unsigned int pin_b;
    unsigned int pin_sw;
    int last_a = 0;
    int last_b = 0;
    int last_sw = 1;
    int quad_state = 0;
    int64_t last_button_us = 0;

    static constexpr int QUAD_STATES[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

public:
    explicit LocalControl(int pi);
    bool poll(double& target_rpm, int& target_pwm_pct, bool& pi_mode);
};

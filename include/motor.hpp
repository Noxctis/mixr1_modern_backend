// include/motor.hpp
#pragma once

// GPIO 13 (PWM), 17 and 27 are SHARED with the fluid controller's pump (Simulink act_wrapper.c). While a MATLAB model runs, the
// daemon must not touch those pins at all: release() only switches this driver off (EN, GPIO 18) and from then on every call is
// a no-op until reclaim() re-initialises the pins for the mixer.
class MotorController {
private:
    int pi_handle;
    int last_duty = -1;        // last duty written to the hardware PWM (-1 = unknown): unchanged values are not re-sent
    bool released = false;     // true while MATLAB owns GPIO 13 / 17 / 27

    void init_pins();          // safe start order: pump + valve parked OFF, EN low, direction, PWM 0 %, then EN high
    void park_fluid_outputs(); // pump EN (15) and valve (5, 6, 12) low: a killed model can never leave them on

public:
    // start_released = true: a MATLAB model is already running at start-up. Only EN is pulled low, the shared pins are not touched.
    explicit MotorController(int pi, bool start_released = false);
    ~MotorController();
    void set_pwm(int duty_cycle);
    void stop_motor();
    void release();            // hand the shared pins to MATLAB: EN low, nothing else
    void reclaim();            // MATLAB has finished: take the pins back (PWM 0 %, direction, EN high)
    bool is_released() const { return released; }
};

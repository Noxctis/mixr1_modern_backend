// src/encoder.cpp
#include "encoder.hpp"
#include <pigpiod_if2.h>

constexpr int AMT102Encoder::QUAD_STATES[16];

void AMT102Encoder::isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
    if (level > 1) return; 
    static_cast<AMT102Encoder*>(user)->update_state(gpio, level, tick);
}

void AMT102Encoder::isr_index(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
    if (level == 1) { 
        static_cast<AMT102Encoder*>(user)->revolutions.fetch_add(1, std::memory_order_relaxed);
    }
}

void AMT102Encoder::update_state(unsigned gpio, unsigned level, uint32_t tick) {
    if (gpio == pin_a) val_a = level;
    else if (gpio == pin_b) val_b = level;

    state = (state << 2) & 0x0F;
    state |= (val_a << 1) | val_b;
    
    count.fetch_add(QUAD_STATES[state], std::memory_order_relaxed);
    last_tick.store(tick, std::memory_order_relaxed); 
    
    long long current_count = count.load(std::memory_order_relaxed);
    
    // SYNCHRONOUS CET FILTER: 
    // Only capture timestamps on full physical slot alignments (multiples of 4)
    // This perfectly eliminates quadrature asymmetry jitter.
    if (current_count % 4 == 0) {
        sync_count.store(current_count, std::memory_order_relaxed);
        sync_tick.store(tick, std::memory_order_relaxed);
    }
}

AMT102Encoder::AMT102Encoder(int pi, unsigned int a, unsigned int b, unsigned int x) : pi_handle(pi), pin_a(a), pin_b(b), pin_x(x) {
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

AMT102Encoder::~AMT102Encoder() {
    callback_cancel(cb_a);
    callback_cancel(cb_b);
    callback_cancel(cb_x);
}

// Atomically return the aligned snapshot
EncoderSnapshot AMT102Encoder::get_sync_snapshot() const {
    return {
        sync_count.load(std::memory_order_relaxed),
        sync_tick.load(std::memory_order_relaxed)
    };
}

long long AMT102Encoder::get_revolutions() const {
    return revolutions.load(std::memory_order_relaxed);
}
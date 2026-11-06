// src/encoder.cpp
#include "encoder.hpp"
#include <pigpiod_if2.h>
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include "config.hpp"

constexpr int AMT102Encoder::QUAD_STATES[16];

void AMT102Encoder::isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
    if (level > 1) return; 
    static_cast<AMT102Encoder*>(user)->update_state(gpio, level, tick);
}

// Index (X) pulse = one full turn. pigpiod delivers the A, B and X callbacks in order from one thread, so `count` here is exactly
// where the shaft was when the index edge arrived. Each pulse is checked against the A/B count before it is counted:
//   moved <= 4 counts since the last accepted pulse  -> the same index mark again: crossed the OTHER way = -1 turn (shaft reversed),
//                                                       same way = a double edge, ignored
//   moved < 0.75 turn                                -> noise / a double edge: rejected (and counted as a warning)
//   moved ~N turns                                   -> N revolutions (+ or - by direction); N > 1 means index pulses were missed:
//                                                       the turns are still counted (from A/B), and the misses are reported
void AMT102Encoder::isr_index(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user) {
    if (level != 1) return;
    auto* self = static_cast<AMT102Encoder*>(user);
    const long long now_count = self->count.load(std::memory_order_relaxed);
    const long long delta = now_count - self->last_index_count;
    const double turns = std::abs(static_cast<double>(delta)) / Config::ENCODER_CPR;
    if (std::llabs(delta) <= 4) {                                 // the same index mark again
        if (self->index_seen && self->last_step_dir != 0 && self->last_step_dir != self->last_cross_dir) {
            self->last_cross_dir = self->last_step_dir;           // shaft reversed back over the mark: one turn the other way
            self->revolutions.fetch_add(self->last_step_dir * Config::ENCODER_DIRECTION, std::memory_order_relaxed);
        }
        return;                                                   // otherwise a double edge / floating line while stopped
    }
    long long n = 1;
    if (self->index_seen) {
        if (turns < 0.75) { self->index_rejected.fetch_add(1, std::memory_order_relaxed); return; }
        n = std::max(1L, std::lround(turns));
        if (n > 1) self->index_missed.fetch_add(n - 1, std::memory_order_relaxed);
    }
    self->index_seen = true;
    self->last_cross_dir = (delta >= 0) ? 1 : -1;
    const int dir = self->last_cross_dir * Config::ENCODER_DIRECTION;
    self->revolutions.fetch_add(dir * n, std::memory_order_relaxed);
    self->last_index_count = now_count;
}

void AMT102Encoder::update_state(unsigned gpio, unsigned level, uint32_t tick) {
    if (gpio == pin_a) val_a = level;
    else if (gpio == pin_b) val_b = level;

    state = (state << 2) & 0x0F;
    state |= (val_a << 1) | val_b;
    
    count.fetch_add(QUAD_STATES[state], std::memory_order_relaxed);
    if (QUAD_STATES[state] != 0) last_step_dir = QUAD_STATES[state];
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
    set_pull_up_down(pi_handle, pin_x, PI_PUD_UP);    // was OFF: a floating index line counted noise as turns
    
    last_index_count = 0;                            // count starts at 0: the first index pulse needs real movement to be accepted
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
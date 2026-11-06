// include/encoder.hpp
#pragma once
#include <atomic>
#include <cstdint>

struct EncoderSnapshot {
    long long count;
    uint32_t tick;
};

class AMT102Encoder {
private:
    int pi_handle;
    unsigned int pin_a, pin_b, pin_x;
    int cb_a, cb_b, cb_x;
    
    std::atomic<long long> count{0};
    std::atomic<uint32_t> last_tick{0};
    
    // NEW: Synchronous CET tracker
    std::atomic<long long> sync_count{0}; 
    std::atomic<uint32_t> sync_tick{0};   
    
    std::atomic<long long> revolutions{0};          // index (X) pulses that passed the check below, signed by direction
    // Index validation: a pulse only counts if the A/B count has moved most of a turn since the last accepted pulse.
    long long last_index_count = 0;                  // A/B count at the last accepted index pulse (callback thread only)
    bool index_seen = false;
    int last_step_dir = 0;                           // direction of the latest A/B step (+1 / -1), callback thread only
    int last_cross_dir = 0;                          // direction of the last counted index crossing
    std::atomic<long long> index_rejected{0};        // pulses rejected as noise (no real turn behind them)
    std::atomic<long long> index_missed{0};          // turns the A/B count saw without an index pulse
    
    uint8_t state = 0;
    uint8_t val_a = 0;
    uint8_t val_b = 0;
    
    static constexpr int QUAD_STATES[16] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    static void isr_router(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);
    static void isr_index(int pi, unsigned gpio, unsigned level, uint32_t tick, void *user);
    void update_state(unsigned gpio, unsigned level, uint32_t tick);

public:
    AMT102Encoder(int pi, unsigned int a, unsigned int b, unsigned int x);
    ~AMT102Encoder();
    
    [[nodiscard]] EncoderSnapshot get_sync_snapshot() const; // NEW: Retrieves the aligned snapshot
    [[nodiscard]] long long get_revolutions() const;
    [[nodiscard]] long long get_index_rejected() const { return index_rejected.load(std::memory_order_relaxed); }
    [[nodiscard]] long long get_index_missed() const { return index_missed.load(std::memory_order_relaxed); }
};
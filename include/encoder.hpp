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
    
    std::atomic<long long> revolutions{0};
    
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
};
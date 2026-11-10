// include/encoder.hpp
#pragma once
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <vector>

struct EncoderSnapshot {
    long long count;     // aligned (multiple-of-4) position count, sign-extended from 32 bits: use wrapped_delta() for differences
    uint32_t tick;       // pigpio time stamp (us) of that edge
};

// Difference of two snapshot counts that stays correct if the 32-bit count ever wraps (after ~2.1e9 counts).
inline long long wrapped_delta(long long now, long long before) {
    return static_cast<long long>(static_cast<int32_t>(static_cast<uint32_t>(now) - static_cast<uint32_t>(before)));
}

// The four edge registers of the paper's synchronous CET (Anuchin et al., Table I): e[p] = (count, tick) of the most recent edge whose
// position count is congruent to p modulo 4, i.e. the most recent edge of each of the four types (A rising, B rising, A falling, B falling).
struct EncoderEdges {
    EncoderSnapshot e[4];
};

// One accepted index (X) pulse, kept for offline analysis (independent speed reference and a check of counts-per-revolution).
struct IndexEvent {
    uint32_t tick;       // pigpio time stamp of the index edge
    long long count;     // A/B count at that edge
};

class AMT102Encoder {
private:
    int pi_handle;
    unsigned int pin_a, pin_b, pin_x;
    int cb_a, cb_b, cb_x;
    
    std::atomic<long long> count{0};
    std::atomic<uint32_t> last_tick{0};
    
    // Synchronous CET tracker. The aligned count (low 32 bits) and its time stamp are packed into ONE 64-bit atomic word, so the
    // control loop can never read a new count with an old tick (torn read). Single writer (callback thread), wait-free reader.
    // pack = (uint64(uint32 count) << 32) | tick
    std::atomic<uint64_t> sync_pack{0};
    static_assert(std::atomic<uint64_t>::is_always_lock_free, "64-bit atomic is not lock-free on this target (add -latomic or use -march=armv7-a)");

    std::atomic<uint64_t> edge_pack[4];              // per-phase (count low 32 bits << 32 | tick), written on every valid A/B step

    // Diagnostics for the PPR study (all written by the callback thread only, read anywhere).
    std::atomic<long long> edge_total{0};            // every A/B callback delivered to this process
    std::atomic<long long> quad_errors{0};           // callbacks whose decoded transition was invalid or unchanged = an edge was lost on the way
    static constexpr size_t INDEX_LOG_SIZE = 65536;
    std::vector<IndexEvent> index_log;               // accepted index pulses (sized once in the constructor; callback thread writes, slot published by index_log_n)
    std::atomic<uint32_t> index_log_n{0};
    
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
    [[nodiscard]] EncoderEdges get_edge_snapshot() const;    // the four per-type edge registers (for the newest-edge estimator)
    [[nodiscard]] long long get_revolutions() const;
    [[nodiscard]] long long get_index_rejected() const { return index_rejected.load(std::memory_order_relaxed); }
    [[nodiscard]] long long get_index_missed() const { return index_missed.load(std::memory_order_relaxed); }
    [[nodiscard]] long long get_edge_total() const { return edge_total.load(std::memory_order_relaxed); }
    [[nodiscard]] long long get_quad_errors() const { return quad_errors.load(std::memory_order_relaxed); }
    [[nodiscard]] long long get_count() const { return count.load(std::memory_order_relaxed); }   // raw (non-aligned) A/B count
    // Copy of the accepted index pulses recorded so far (call after the run; at most INDEX_LOG_SIZE entries are kept).
    [[nodiscard]] size_t copy_index_log(IndexEvent* out, size_t max_n) const;
};
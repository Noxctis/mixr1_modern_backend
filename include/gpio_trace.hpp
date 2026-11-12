// include/gpio_trace.hpp
// GPIO trace for integrity and latency: every level change of the chosen GPIO pins is stored with the pigpio time stamp (the 5 us
// sampler clock of the edge itself) and with the host clock at the moment the daemon's callback ran. Offline (analysis/gpio_trace_report.py):
//   integrity : edges of A and B must be equal in number and 512 per revolution (256 PPR), one X edge per revolution, time stamps must not
//               go backwards, no pin may be silent while the others run;
//   latency   : host time minus tick time = how long the edge needed to reach the daemon (jitter = the spread of that difference).
// No extra socket round trip per edge: the offset between the two clocks is removed offline.
// Pins: the inputs of the rig (encoder A, B, X, knob, push button, start key) and the outputs that change slowly (driver enable and
// direction). The hardware PWM (GPIO 13, 20 kHz) and the SPI of the display are NOT traced by default, they would give tens of thousands
// of events per second; "all" adds the PWM pin and the fluid-controller pins.
#pragma once
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>
#include <pigpiod_if2.h>
#include "io_trace.hpp"

class GpioTrace {
public:
    struct Event { uint64_t host_ns; uint32_t tick; uint8_t gpio; uint8_t level; };

    GpioTrace() = default;
    ~GpioTrace() { stop(); }
    GpioTrace(const GpioTrace&) = delete;
    GpioTrace& operator=(const GpioTrace&) = delete;

    bool active() const { return active_; }

    void start(int pi, const std::vector<unsigned>& pins, size_t max_events) {
        if (active_) return;
        pi_ = pi; cap_ = max_events; events_.assign(cap_, Event{});
        count_.store(0); overflow_.store(0);
        t0_ = TRACE_T0;                                         // same zero as io_trace.hpp
        for (unsigned p : pins) {
            const int id = callback_ex(pi_, p, EITHER_EDGE, &GpioTrace::isr, this);
            if (id >= 0) { cb_.push_back(id); pins_.push_back(p); }
        }
        active_ = !cb_.empty();
    }

    void stop() {
        if (!active_) return;
        for (int id : cb_) callback_cancel(static_cast<unsigned>(id));
        cb_.clear(); active_ = false;
    }

    // Writes <dir>/gpio_trace_<stamp>.csv (seq, host_s, tick_us, gpio, level) and a one-line-per-pin summary. Returns the CSV path ("" on failure).
    std::string save(const std::string& dir, const std::string& stamp) {
        stop();
        const size_t n = count_.load(std::memory_order_acquire);
        if (n == 0) return "";
        std::error_code ec; std::filesystem::create_directories(dir, ec);
        const std::string path = dir + "/gpio_trace_" + stamp + ".csv";
        FILE* f = std::fopen(path.c_str(), "w");
        if (!f) return "";
        std::fprintf(f, "seq,host_s,tick_us,gpio,level\n");
        for (size_t i = 0; i < n; ++i) {
            const Event& e = events_[i];
            std::fprintf(f, "%zu,%.6f,%u,%u,%u\n", i, static_cast<double>(e.host_ns) * 1e-9, e.tick, e.gpio, e.level);
        }
        const bool ok = std::fclose(f) == 0;
        std::printf("[GPIO] trace: %zu events saved to %s (%llu lost: buffer full)\n", n, path.c_str(), static_cast<unsigned long long>(overflow_.load()));
        return ok ? path : "";
    }

    size_t events() const { return count_.load(std::memory_order_relaxed); }
    unsigned long long overflow() const { return overflow_.load(std::memory_order_relaxed); }

private:
    static void isr(int, unsigned gpio, unsigned level, uint32_t tick, void* user) {
        if (level > 1) return;                                       // watchdog time-out, not an edge
        static_cast<GpioTrace*>(user)->push(gpio, level, tick);
    }
    void push(unsigned gpio, unsigned level, uint32_t tick) {
        const size_t n = count_.load(std::memory_order_relaxed);       // single writer: the pigpio callback thread
        if (n >= cap_) { overflow_.fetch_add(1, std::memory_order_relaxed); return; }
        const auto dt = std::chrono::steady_clock::now() - t0_;
        events_[n] = Event{static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count()), tick,
                           static_cast<uint8_t>(gpio), static_cast<uint8_t>(level)};
        count_.store(n + 1, std::memory_order_release);
    }

    int pi_ = -1;
    bool active_ = false;
    size_t cap_ = 0;
    std::vector<Event> events_;
    std::atomic<size_t> count_{0};
    std::atomic<unsigned long long> overflow_{0};
    std::chrono::steady_clock::time_point t0_{};
    std::vector<int> cb_;
    std::vector<unsigned> pins_;
};

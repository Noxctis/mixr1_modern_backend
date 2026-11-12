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
#include <thread>
#include <vector>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#endif
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

    // Streams every edge to <path> while the run goes on (a background thread drains a ring buffer of max_events every 100 ms), so the trace can last
    // as long as the run: nothing waits in memory until the end. Events are lost only if the disk stalls for longer than the ring holds.
    void start(int pi, const std::vector<unsigned>& pins, size_t max_events, const std::string& path) {
        if (active_) return;
        file_ = std::fopen(path.c_str(), "w");
        if (!file_) return;
        path_ = path;
        std::fprintf(file_, "seq,host_s,tick_us,gpio,level\n");
        pi_ = pi; cap_ = max_events; events_.assign(cap_, Event{});
        count_.store(0); read_.store(0); overflow_.store(0);
        t0_ = TRACE_T0;                                         // same zero as io_trace.hpp
        for (unsigned p : pins) {
            const int id = callback_ex(pi_, p, EITHER_EDGE, &GpioTrace::isr, this);
            if (id >= 0) { cb_.push_back(id); pins_.push_back(p); }
        }
        active_ = !cb_.empty();
        if (!active_) { std::fclose(file_); file_ = nullptr; return; }
        done_.store(false);
        writer_ = std::thread([this]() { drain_loop(); });
    }

    void stop() {
        if (!active_) return;
        for (int id : cb_) callback_cancel(static_cast<unsigned>(id));
        cb_.clear(); active_ = false;
        done_.store(true);
        if (writer_.joinable()) writer_.join();                  // the writer drains what is left before it returns
        if (file_) { std::fclose(file_); file_ = nullptr; }
        std::printf("[GPIO] trace: %zu events saved to %s (%llu lost: ring buffer overrun)\n", count_.load(), path_.c_str(), static_cast<unsigned long long>(overflow_.load()));
    }

    // Ends the trace (the file is already on disk). Arguments kept for the old call; returns the CSV path.
    std::string save(const std::string&, const std::string&) { stop(); return path_; }
    const std::string& path() const { return path_; }

    size_t events() const { return count_.load(std::memory_order_relaxed); }
    unsigned long long overflow() const { return overflow_.load(std::memory_order_relaxed); }

private:
    static void isr(int, unsigned gpio, unsigned level, uint32_t tick, void* user) {
        if (level > 1) return;                                       // watchdog time-out, not an edge
        static_cast<GpioTrace*>(user)->push(gpio, level, tick);
    }
    void push(unsigned gpio, unsigned level, uint32_t tick) {
        const size_t n = count_.load(std::memory_order_relaxed);       // single writer: the pigpio callback thread
        if (n - read_.load(std::memory_order_acquire) >= cap_) { overflow_.fetch_add(1, std::memory_order_relaxed); return; }   // the file writer is behind
        const auto dt = std::chrono::steady_clock::now() - t0_;
        events_[n % cap_] = Event{static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(dt).count()), tick,
                           static_cast<uint8_t>(gpio), static_cast<uint8_t>(level)};
        count_.store(n + 1, std::memory_order_release);
    }

    void drain_loop() {
#ifdef __linux__
        cpu_set_t others; CPU_ZERO(&others);                       // off the control core, normal scheduling
        for (int c = 0; c < 3; ++c) CPU_SET(c, &others);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &others);
        sched_param normal{}; normal.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal);
#endif
        std::string buf;
        while (true) {
            const bool last = done_.load();                         // read the flag BEFORE the count: everything pushed before stop() is then drained
            const size_t c = count_.load(std::memory_order_acquire);
            size_t r = read_.load(std::memory_order_relaxed);
            buf.clear(); char line[80];
            for (size_t i = r; i < c; ++i) {
                const Event& e = events_[i % cap_];
                const int len = std::snprintf(line, sizeof line, "%zu,%.6f,%u,%u,%u\n", i, static_cast<double>(e.host_ns) * 1e-9, e.tick, e.gpio, e.level);
                if (len > 0) buf.append(line, static_cast<size_t>(len));
            }
            if (!buf.empty() && file_) { std::fwrite(buf.data(), 1, buf.size(), file_); std::fflush(file_); }
            read_.store(c, std::memory_order_release);
            if (last) return;
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    int pi_ = -1;
    std::FILE* file_ = nullptr;
    std::string path_;
    std::thread writer_;
    std::atomic<bool> done_{false};
    std::atomic<size_t> read_{0};
    bool active_ = false;
    size_t cap_ = 0;
    std::vector<Event> events_;
    std::atomic<size_t> count_{0};
    std::atomic<unsigned long long> overflow_{0};
    std::chrono::steady_clock::time_point t0_{};
    std::vector<int> cb_;
    std::vector<unsigned> pins_;
};

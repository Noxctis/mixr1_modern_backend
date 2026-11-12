// include/io_trace.hpp
// Trace of the non-GPIO channels of the daemon, for integrity and latency (option --trace-io):
//   MOTOR_PWM    every hardware_PWM() call to the motor driver (a socket round trip to pigpiod): when, how long it took, whether it succeeded, the duty
//   TORQUE_READ  every Modbus-RTU read of the torque amplifier (request + reply on the RS485 line): when, how long, success or the kind of failure, the raw count
//   TORQUE_ZERO  every zero (tare) command to the amplifier
// The time stamps use the same clock as include/gpio_trace.hpp (seconds since the start of the process), so the two CSV files can be laid side by side.
// The level sensor (ToF) is not read by this daemon (it belongs to the Simulink model / the vl53l4cd tools), so it is not traced here.
#pragma once
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <string>
#include <system_error>
#include <vector>

inline const std::chrono::steady_clock::time_point TRACE_T0 = std::chrono::steady_clock::now();   // shared zero of every trace

class IoTrace {
public:
    enum Channel : uint8_t { MOTOR_PWM = 0, TORQUE_READ = 1, TORQUE_ZERO = 2 };
    // err: 0 none, 1 no / short reply (timeout), 2 wrong header, 3 CRC error, 4 write failed / port not open, 5 cannot decode
    struct Rec { uint64_t start_ns; uint32_t dur_us; int32_t value; uint8_t ch; uint8_t ok; uint8_t err; };

    void enable(size_t cap) { recs_.assign(cap, Rec{}); n_.store(0); on_.store(true); }
    bool on() const { return on_.load(std::memory_order_relaxed); }

    // t0 = clock before the call; the duration is taken now. Safe from several threads (one slot per call).
    void push(Channel ch, std::chrono::steady_clock::time_point t0, bool ok, int32_t value, uint8_t err = 0) {
        const auto t1 = std::chrono::steady_clock::now();
        const size_t i = n_.fetch_add(1, std::memory_order_relaxed);
        if (i >= recs_.size()) return;
        recs_[i] = Rec{static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(t0 - TRACE_T0).count()),
                       static_cast<uint32_t>(std::chrono::duration_cast<std::chrono::microseconds>(t1 - t0).count()), value,
                       static_cast<uint8_t>(ch), static_cast<uint8_t>(ok ? 1 : 0), err};
    }

    // Call after the threads that push have stopped. Writes <dir>/io_trace_<stamp>.csv (channel, start_s, dur_us, ok, value, err).
    std::string save(const std::string& dir, const std::string& stamp) {
        on_.store(false);
        const size_t n = std::min(n_.load(), recs_.size());
        if (n == 0) return "";
        std::vector<Rec> v(recs_.begin(), recs_.begin() + static_cast<std::ptrdiff_t>(n));
        std::sort(v.begin(), v.end(), [](const Rec& a, const Rec& b) { return a.start_ns < b.start_ns; });
        std::error_code ec; std::filesystem::create_directories(dir, ec);
        const std::string path = dir + "/io_trace_" + stamp + ".csv";
        FILE* f = std::fopen(path.c_str(), "w");
        if (!f) return "";
        static const char* const names[] = {"motor_pwm", "torque_read", "torque_zero"};
        std::fprintf(f, "channel,start_s,dur_us,ok,value,err\n");
        for (const Rec& r : v) std::fprintf(f, "%s,%.6f,%u,%u,%d,%u\n", names[r.ch < 3 ? r.ch : 0], static_cast<double>(r.start_ns) * 1e-9, r.dur_us, r.ok, r.value, r.err);
        const bool ok = std::fclose(f) == 0;
        std::printf("[IO] trace: %zu calls saved to %s%s\n", n, path.c_str(), n_.load() > recs_.size() ? " (buffer full: later calls lost)" : "");
        return ok ? path : "";
    }

private:
    std::vector<Rec> recs_;
    std::atomic<size_t> n_{0};
    std::atomic<bool> on_{false};
};

inline IoTrace io_trace;

// include/run_recorder.hpp
// Black-box recorder: every run of the daemon (Mode 1, Mode 2 or open loop, whatever the dashboard does) saves a complete record of its own
// in run_records/<date>_<time>[_<label>]/ on the Raspberry Pi, independent of the dashboard and of the other logs:
//   manifest.txt   the whole configuration at the start: command line, build time, host, Kp, Ki, feedback filter, estimator, counts per revolution,
//                  window, loop period, power limits, slew, stall, torque amplifier settings, impeller options, the water temperature and note you pass;
//                  at the end: end time, rows, and the latency / integrity summary
//   daemon.log     everything the daemon prints (index-pulse warnings, torque zero messages, safety trips, dashboard commands), each line time-stamped
//   telemetry.csv  one row per control loop (100 Hz) in every mode: speeds, duty, target, torque, power, gains in force, mode, loop period,
//                  wake lateness, processing time of the loop, age of the torque and power readings used in that loop
//   summary.txt    LATENCY AND INTEGRITY, measured during the run and written when it ends (also printed and copied to the end of manifest.txt):
//                  - control loop: period (median, p99, max), wake lateness (how late the OS woke the loop after its scheduled time), processing time,
//                    loops later than 1.5x / 2x the period, estimated lost loops
//                  - torque reading over RS485: requests ok / failed, rate against the period, age of the value the loop used (median, p99, max),
//                    longest time without a good reading
//                  - power meter (I2C): same
//                  - dashboard socket: packets sent / not sent
//                  - telemetry rows lost by the recorder itself (rows made vs written)
// Options: --no-record (off), --run-label=TEXT (added to the folder name), --note=TEXT, --water-temp=C (stored in the manifest).
#pragma once
#include <algorithm>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <mutex>
#include <sstream>
#include <streambuf>
#include <string>
#include <system_error>
#include <thread>
#include <vector>
#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <unistd.h>
#endif

class RunRecorder {
public:
    static constexpr const char* HEADER =
        "t_s,loop_dt_ms,wake_late_us,work_us,mode,impeller,target_rpm,raw_rpm,ema_rpm,feedback_rpm,revolutions,pwm_counts,pwm_pct,torque_nm,torque_age_ms,"
        "bus_v,current_ma,elec_power_w,power_age_ms,kp,ki,alpha,power_limiting,tare_phase,dashboard_connected";
    // mode: 0 standalone stopped, 1 standalone running (PI), 2 dashboard PI, 3 dashboard open loop; age = -1 until the first good reading

    ~RunRecorder() { stop(); }
    bool active() const { return active_; }
    const std::string& dir() const { return dir_; }

    // Creates the folder, opens the files and starts copying std::cout into daemon.log. Returns false (and records nothing) on failure.
    bool start(const std::string& root, const std::string& label, int argc, char** argv) {
        if (active_) return true;
        std::time_t t = std::time(nullptr); std::tm tmv{};
#ifdef _WIN32
        localtime_s(&tmv, &t);
#else
        localtime_r(&t, &tmv);
#endif
        char b[32]; std::strftime(b, sizeof b, "%Y%m%d_%H%M%S", &tmv);
        std::string name = b; if (!label.empty()) name += "_" + sanitize(label);
        dir_ = root + "/" + name;
        std::error_code ec; std::filesystem::create_directories(dir_, ec);
        if (ec) return false;
        manifest_ = std::fopen((dir_ + "/manifest.txt").c_str(), "w");
        telemetry_ = std::fopen((dir_ + "/telemetry.csv").c_str(), "w");
        if (!manifest_ || !telemetry_) { close_all(); return false; }
        std::fprintf(telemetry_, "%s\n", HEADER);
        t0_ = std::chrono::steady_clock::now();
        tee_.open(dir_ + "/daemon.log", &t0_);
        old_buf_ = std::cout.rdbuf(&tee_);
        tee_.set_target(old_buf_);
        std::string cmd; for (int i = 0; i < argc; ++i) { if (i) cmd += ' '; cmd += argv[i]; }
        kv("run_folder", name); kv("start_time", b); kv("command_line", cmd);
        kv("build", std::string(__DATE__) + " " + __TIME__);
#ifdef __linux__
        char host[128] = {0}; if (gethostname(host, sizeof host - 1) == 0) kv("host", host);
#endif
        active_ = true;
        writer_ = std::thread([this]() { run_writer(); });
        return true;
    }

    void kv(const std::string& key, const std::string& value) {
        if (manifest_) { std::fprintf(manifest_, "%s=%s\n", key.c_str(), value.c_str()); std::fflush(manifest_); }
    }
    void kv(const std::string& key, double value) { char b[64]; std::snprintf(b, sizeof b, "%.6g", value); kv(key, std::string(b)); }

    // nominal control-loop period and the polling period of the torque reader (ms), for the lost-loop and rate estimates
    void set_nominal(double loop_ms, double torque_period_ms) { nominal_ms_ = loop_ms; torque_period_ms_ = torque_period_ms; }

    // ---- events from other threads: one call per request, ok = a valid answer came back ----
    void note_torque(bool ok) { note(ok, torque_ok_, torque_fail_, torque_last_ns_, torque_gap_max_ns_); }
    void note_power(bool ok)  { note(ok, power_ok_,  power_fail_,  power_last_ns_,  power_gap_max_ns_); }
    void note_dash(bool sent) { (sent ? dash_sent_ : dash_fail_).fetch_add(1, std::memory_order_relaxed); }

    // One control loop. Cheap (one snprintf and an append); the file is written by a background thread.
    //   wake_late_us = wake-up time minus the scheduled time (scheduling latency); work_us = from wake-up until this call (reading, PI, PWM write)
    void row(double loop_dt_ms, double wake_late_us, double work_us, int mode, int impeller, double target, double raw, double ema, double fb, long long revs,
             int pwm_counts, double torque, double bus_v, double current_ma, double elec_w, double kp, double ki, double alpha, bool limiting, int tare_phase,
             bool dash) {
        if (!active_) return;
        const auto now = std::chrono::steady_clock::now();
        if (prev_cost_us_ >= 0) h_cost_.add(prev_cost_us_);       // what the previous call of row() cost the control thread
        const double t = std::chrono::duration<double>(now - t0_).count();
        const long long now_ns = ns_of(now);
        const double tq_age = age_ms(torque_last_ns_, now_ns), pw_age = age_ms(power_last_ns_, now_ns);
        h_dt_.add(loop_dt_ms); h_late_.add(wake_late_us); h_work_.add(work_us);
        if (tq_age >= 0) h_tqage_.add(tq_age);
        if (pw_age >= 0) h_pwage_.add(pw_age);
        if (!std::isfinite(torque)) ++torque_nan_rows_;
        if (nominal_ms_ > 0 && loop_dt_ms > 1.5 * nominal_ms_) { ++late15_; lost_est_ += std::max(0, static_cast<int>(std::lround(loop_dt_ms / nominal_ms_)) - 1); }
        if (nominal_ms_ > 0 && loop_dt_ms > 2.0 * nominal_ms_) ++late20_;
        ++made_;
        char b[520];
        const int n = std::snprintf(b, sizeof b, "%.3f,%.2f,%.0f,%.0f,%d,%d,%.2f,%.3f,%.3f,%.3f,%lld,%d,%.2f,%.4f,%.1f,%.3f,%.1f,%.3f,%.1f,%.4f,%.4f,%.3f,%d,%d,%d\n",
                                    t, loop_dt_ms, wake_late_us, work_us, mode, impeller, target, raw, ema, fb, revs, pwm_counts, pwm_counts * 100.0 / 4095.0, torque,
                                    tq_age, bus_v, current_ma, elec_w, pw_age, kp, ki, alpha, limiting ? 1 : 0, tare_phase, dash ? 1 : 0);
        if (n <= 0) return;
        {
            std::lock_guard<std::mutex> lk(mtx_);
            pending_.append(b, static_cast<size_t>(std::min<int>(n, static_cast<int>(sizeof b) - 1)));
            ++rows_;
        }
        prev_cost_us_ = std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - now).count();
    }

    void stop() {
        if (!active_) return;
        active_ = false;
        { std::lock_guard<std::mutex> lk(mtx_); stop_ = true; }
        cv_.notify_all();
        if (writer_.joinable()) writer_.join();
        std::time_t t = std::time(nullptr); std::tm tmv{};
#ifdef _WIN32
        localtime_s(&tmv, &t);
#else
        localtime_r(&t, &tmv);
#endif
        char b[32]; std::strftime(b, sizeof b, "%Y%m%d_%H%M%S", &tmv);
        kv("end_time", b); kv("telemetry_rows", static_cast<double>(rows_));
        const std::string s = summary();
        if (std::FILE* f = std::fopen((dir_ + "/summary.txt").c_str(), "w")) { std::fputs(s.c_str(), f); std::fclose(f); }
        if (manifest_) std::fprintf(manifest_, "\n%s", s.c_str());
        std::cout << s << "[RECORD] Run saved to " << dir_ << "/ (manifest.txt, daemon.log, telemetry.csv, summary.txt)\n";
        if (old_buf_) { std::cout.rdbuf(old_buf_); old_buf_ = nullptr; }
        tee_.close();
        close_all();
    }

private:
    // Fixed-width histogram: quantiles without storing every sample (the run can last hours).
    struct Hist {
        double w; std::vector<unsigned> bins; unsigned long long n = 0, over = 0; double mx = 0, sum = 0;
        Hist(double width, size_t count) : w(width), bins(count, 0) {}
        void add(double x) {
            if (!std::isfinite(x)) return;
            ++n; sum += x; if (x > mx) mx = x;
            const double i = x / w;
            if (i < 0) ++bins[0]; else if (i >= static_cast<double>(bins.size())) ++over; else ++bins[static_cast<size_t>(i)];
        }
        double q(double p) const {
            if (!n) return std::nan("");
            const unsigned long long target = static_cast<unsigned long long>(std::ceil(p * static_cast<double>(n)));
            unsigned long long c = 0;
            for (size_t i = 0; i < bins.size(); ++i) { c += bins[i]; if (c >= target) return (i + 1) * w; }
            return mx;                                             // in the overflow bin: the true maximum is the best statement
        }
        double mean() const { return n ? sum / static_cast<double>(n) : std::nan(""); }
    };

    static long long ns_of(std::chrono::steady_clock::time_point t) { return std::chrono::duration_cast<std::chrono::nanoseconds>(t.time_since_epoch()).count(); }
    static double age_ms(const std::atomic<long long>& last, long long now_ns) { const long long l = last.load(std::memory_order_relaxed); return l ? (now_ns - l) / 1.0e6 : -1.0; }
    static void note(bool ok, std::atomic<unsigned long long>& okc, std::atomic<unsigned long long>& failc, std::atomic<long long>& last, std::atomic<long long>& gap_max) {
        if (!ok) { failc.fetch_add(1, std::memory_order_relaxed); return; }
        const long long now = ns_of(std::chrono::steady_clock::now());
        const long long prev = last.exchange(now);
        if (prev) { const long long g = now - prev; long long m = gap_max.load(); while (g > m && !gap_max.compare_exchange_weak(m, g)) {} }
        okc.fetch_add(1, std::memory_order_relaxed);
    }

    static std::string fmt(const char* f, double a, double b2 = 0, double c = 0) { char b[200]; std::snprintf(b, sizeof b, f, a, b2, c); return b; }

    std::string summary() const {
        std::ostringstream o;
        const double dur = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0_).count();
        o << "=== LATENCY AND INTEGRITY SUMMARY (measured by the daemon during this run) ===\n";
        o << "duration " << fmt("%.1f", dur) << " s, control loops " << made_ << ", nominal period " << fmt("%.1f", nominal_ms_) << " ms\n";
        if (h_dt_.n) {
            o << "control loop period : median " << fmt("%.2f", h_dt_.q(0.5)) << " ms, p99 " << fmt("%.2f", h_dt_.q(0.99)) << " ms, max " << fmt("%.2f", h_dt_.mx)
              << " ms, mean " << fmt("%.3f", h_dt_.mean()) << " ms\n";
            o << "wake-up lateness    : median " << fmt("%.0f", h_late_.q(0.5)) << " us, p99 " << fmt("%.0f", h_late_.q(0.99)) << " us, max " << fmt("%.0f", h_late_.mx)
              << " us (OS scheduling latency of the control thread)\n";
            o << "processing per loop : median " << fmt("%.0f", h_work_.q(0.5)) << " us, p99 " << fmt("%.0f", h_work_.q(0.99)) << " us, max " << fmt("%.0f", h_work_.mx)
              << " us (wake-up until the loop's PI / PWM step is done)\n";
            o << "loops late          : " << late15_ << " longer than 1.5x the period, " << late20_ << " longer than 2x, estimated lost loops " << lost_est_
              << " (" << fmt("%.3f", made_ ? 100.0 * lost_est_ / static_cast<double>(made_ + lost_est_) : 0.0) << " %)\n";
        }
        auto device = [&](const char* name, const char* bus, unsigned long long ok, unsigned long long bad, long long gap_ns, const Hist& age, double period_ms) {
            const unsigned long long tot = ok + bad;
            o << name << " (" << bus << "): ";
            if (!tot) { o << "no request made in this run\n"; return; }
            o << tot << " requests, " << bad << " failed (" << fmt("%.2f", 100.0 * bad / static_cast<double>(tot)) << " %)";
            if (period_ms > 0 && dur > 0) o << ", " << fmt("%.1f", ok / dur) << " good readings/s against " << fmt("%.1f", 1000.0 / period_ms) << " expected";
            o << "\n    age of the value used by the loop: median " << fmt("%.1f", age.q(0.5)) << " ms, p99 " << fmt("%.1f", age.q(0.99)) << " ms, max " << fmt("%.1f", age.mx)
              << " ms; longest time between good readings " << fmt("%.1f", gap_ns / 1.0e6) << " ms\n";
        };
        device("torque amplifier", "RS485 Modbus", torque_ok_.load(), torque_fail_.load(), torque_gap_max_ns_.load(), h_tqage_, torque_period_ms_);
        o << "    loops with no torque value (NaN: no sensor or dropout): " << torque_nan_rows_ << " of " << made_ << "\n";
        device("power meter", "I2C", power_ok_.load(), power_fail_.load(), power_gap_max_ns_.load(), h_pwage_, 0.0);
        o << "dashboard socket: " << dash_sent_.load() << " packets sent, " << dash_fail_.load() << " not sent (client absent, slow or disconnected)\n";
        o << "recorder cost to the control thread (one row): median " << fmt("%.0f", h_cost_.q(0.5)) << " us, p99 " << fmt("%.0f", h_cost_.q(0.99)) << " us, max " << fmt("%.0f", h_cost_.mx) << " us\n";
        o << "recorder: " << made_ << " rows made, " << rows_ << " queued for the file";
        o << (made_ == rows_ ? ", none lost\n" : ", MISMATCH\n");
        o << "Hardware level (encoder edges, driver writes, torque request latency) is measured by --trace-gpio and --trace-io, not here.\n";
        return o.str();
    }

    // Copies everything written to std::cout into a file, with the time since the start in front of every line.
    class Tee : public std::streambuf {
    public:
        void open(const std::string& path, const std::chrono::steady_clock::time_point* t0) { f_ = std::fopen(path.c_str(), "w"); t0_ = t0; }
        void set_target(std::streambuf* target) { target_ = target; }
        void flush() { if (f_) std::fflush(f_); }
        void close() { if (f_) { std::fflush(f_); std::fclose(f_); f_ = nullptr; } }
    protected:
        int_type overflow(int_type c) override {
            if (c == traits_type::eof()) return traits_type::not_eof(c);
            const char ch = static_cast<char>(c);
            if (target_) target_->sputc(ch);
            if (f_) {
                if (at_line_start_) {
                    std::fprintf(f_, "[%9.3f] ", std::chrono::duration<double>(std::chrono::steady_clock::now() - *t0_).count());
                    at_line_start_ = false;
                }
                std::fputc(ch, f_);
                if (ch == '\n') at_line_start_ = true;      // no flush here: the writer thread flushes every 500 ms, so the control thread never waits for the disk
            }
            return c;
        }
        int sync() override { if (target_) target_->pubsync(); if (f_) std::fflush(f_); return 0; }
    private:
        std::FILE* f_ = nullptr;
        std::streambuf* target_ = nullptr;
        const std::chrono::steady_clock::time_point* t0_ = nullptr;
        bool at_line_start_ = true;
    };

    static std::string sanitize(std::string s) {
        for (char& c : s) if (!(std::isalnum(static_cast<unsigned char>(c)) || c == '_' || c == '-')) c = '_';
        return s;
    }
    void close_all() {
        if (manifest_) { std::fclose(manifest_); manifest_ = nullptr; }
        if (telemetry_) { std::fclose(telemetry_); telemetry_ = nullptr; }
    }
    void run_writer() {
#ifdef __linux__
        cpu_set_t others; CPU_ZERO(&others);                       // off the control core and back to normal scheduling (std::thread inherits them)
        for (int c = 0; c < 3; ++c) CPU_SET(c, &others);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &others);
        sched_param normal{}; normal.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal);
#endif
        std::unique_lock<std::mutex> lk(mtx_);
        while (true) {
            cv_.wait_for(lk, std::chrono::milliseconds(500), [this]() { return stop_; });
            std::string out; out.swap(pending_);
            pending_.reserve(32768);                              // no re-allocation while the control thread appends
            tee_.flush();
            const bool done = stop_;
            lk.unlock();
            if (!out.empty() && telemetry_) { std::fwrite(out.data(), 1, out.size(), telemetry_); std::fflush(telemetry_); }
            if (done) return;
            lk.lock();
        }
    }

    bool active_ = false;
    std::string dir_;
    std::FILE* manifest_ = nullptr;
    std::FILE* telemetry_ = nullptr;
    std::chrono::steady_clock::time_point t0_{};
    Tee tee_;
    std::streambuf* old_buf_ = nullptr;
    std::thread writer_;
    std::mutex mtx_;
    std::condition_variable cv_;
    std::string pending_;
    bool stop_ = false;
    unsigned long long rows_ = 0, made_ = 0;
    // latency and integrity
    double nominal_ms_ = 10.0, torque_period_ms_ = 20.0;
    double prev_cost_us_ = -1.0;
    Hist h_cost_{1.0, 2000}, h_dt_{0.05, 2000}, h_late_{10.0, 5000}, h_work_{10.0, 5000}, h_tqage_{1.0, 5000}, h_pwage_{1.0, 5000};
    unsigned long long late15_ = 0, late20_ = 0, lost_est_ = 0, torque_nan_rows_ = 0;
    std::atomic<unsigned long long> torque_ok_{0}, torque_fail_{0}, power_ok_{0}, power_fail_{0}, dash_sent_{0}, dash_fail_{0};
    std::atomic<long long> torque_last_ns_{0}, torque_gap_max_ns_{0}, power_last_ns_{0}, power_gap_max_ns_{0};
};

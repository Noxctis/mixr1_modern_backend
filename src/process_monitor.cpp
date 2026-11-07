// src/process_monitor.cpp
#include "process_monitor.hpp"
#include "config.hpp"
#include <cctype>
#include <chrono>
#include <cstdio>
#include <dirent.h>
#include <fstream>
#include <pthread.h>
#include <sched.h>
#include <unistd.h>

namespace {

bool ends_with(const std::string& s, const std::string& tail) {
    return s.size() >= tail.size() && s.compare(s.size() - tail.size(), tail.size(), tail) == 0;
}
bool starts_with(const std::string& s, const std::string& head) { return s.rfind(head, 0) == 0; }

// argv[0] of a process ("" for kernel threads, zombies, or a process that just exited)
std::string read_argv0(const std::string& pid) {
    std::ifstream f("/proc/" + pid + "/cmdline", std::ios::binary);
    std::string argv0;
    if (f) std::getline(f, argv0, '\0');
    return argv0;
}

std::string read_comm(const std::string& pid) {
    std::ifstream f("/proc/" + pid + "/comm");
    std::string comm;
    if (f) std::getline(f, comm);
    return comm;
}

bool is_matlab_program(const std::string& argv0) {
    if (argv0.empty()) return false;
    const size_t slash = argv0.find_last_of('/');
    const std::string base = (slash == std::string::npos) ? argv0 : argv0.substr(slash + 1);
    return ends_with(base, ".elf")
        || argv0.find("MATLAB_ws") != std::string::npos
        || starts_with(base, "mixr1_level") || starts_with(base, "mixr_level");
}

}  // namespace

ProcessMonitor::Match ProcessMonitor::scan() {
    Match m;
    DIR* proc = opendir("/proc");
    if (!proc) return m;
    const std::string self = std::to_string(getpid());
    while (dirent* e = readdir(proc)) {
        if (!std::isdigit(static_cast<unsigned char>(e->d_name[0]))) continue;
        const std::string pid = e->d_name;
        if (pid == self) continue;
        const std::string argv0 = read_argv0(pid);
        if (argv0.empty()) continue;                                   // kernel thread / zombie
        if (is_matlab_program(argv0) || read_comm(pid) == "raspberrypi_get") {
            m.found = true;
            m.pid = std::stoi(pid);
            m.program = argv0;
            break;
        }
    }
    closedir(proc);
    return m;
}

void ProcessMonitor::start() {
    if (thread_.joinable()) return;
    stop_ = false;
    {
        const Match first = scan();                      // known before the first control loop, no 200 ms blind spot
        active_ = first.found;
        std::lock_guard<std::mutex> lk(mtx_);
        program_ = first.program;
    }
    thread_ = std::thread([this]() {
        // std::thread inherits the control thread's core 3 + SCHED_FIFO 90: move to the other cores, normal scheduling
        cpu_set_t others;
        CPU_ZERO(&others);
        for (int c = 0; c < 3; ++c) CPU_SET(c, &others);
        pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &others);
        sched_param normal{};
        normal.sched_priority = 0;
        pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal);

        auto last_seen = std::chrono::steady_clock::now();
        while (!stop_.load()) {
            const Match m = scan();
            const auto now = std::chrono::steady_clock::now();
            if (m.found) {
                last_seen = now;
                if (!active_.load()) {
                    { std::lock_guard<std::mutex> lk(mtx_); program_ = m.program; }
                    active_.store(true, std::memory_order_release);          // take-over: at once
                }
            } else if (active_.load() && now - last_seen >= std::chrono::milliseconds(Config::MATLAB_RELEASE_DEBOUNCE_MS)) {
                active_.store(false, std::memory_order_release);             // hand-back: only after the debounce
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(Config::MATLAB_SCAN_PERIOD_MS));
        }
    });
}

void ProcessMonitor::stop() {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
}

std::string ProcessMonitor::program() const {
    std::lock_guard<std::mutex> lk(mtx_);
    return program_;
}

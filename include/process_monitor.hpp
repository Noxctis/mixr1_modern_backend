// include/process_monitor.hpp
#pragma once
#include <atomic>
#include <mutex>
#include <string>
#include <thread>

// Detects a MATLAB / Simulink model running on the Pi (Mode 3).
//
// The Simulink Support Package for Raspberry Pi builds every model into <model>.elf under ~/MATLAB_ws/<release>/... and runs that
// file, in Monitor & Tune as well as in a deployed run. A process counts as a MATLAB run when its program (argv[0]) is:
//   * any *.elf file                        -> every Simulink model, whatever its name
//   * anything under a MATLAB_ws folder
//   * named mixr1_level* or mixr_level*     -> the fluid-level models, even if they were started by hand
//   * (legacy) a process called raspberrypi_get
// Only argv[0] is checked, so "sudo pkill mixr1_level" or a compiler working inside MATLAB_ws never counts.
//
// The scan reads /proc directly (no shell, no fork) in a background thread, off the control core: start-up is seen within
// MATLAB_SCAN_PERIOD_MS, and the end of a run only after MATLAB_RELEASE_DEBOUNCE_MS without a model (a restart in Monitor & Tune
// does not bounce control back and forth).
class ProcessMonitor {
public:
    struct Match {
        bool found = false;
        int pid = 0;
        std::string program;       // argv[0] of the matching process
    };

    [[nodiscard]] static Match scan();                                  // one pass over /proc (any thread)
    [[nodiscard]] static bool is_simulink_running() { return scan().found; }

    ProcessMonitor() = default;
    ~ProcessMonitor() { stop(); }
    ProcessMonitor(const ProcessMonitor&) = delete;
    ProcessMonitor& operator=(const ProcessMonitor&) = delete;

    void start();                                                       // background thread; safe to call from a SCHED_FIFO thread
    void stop();
    [[nodiscard]] bool active() const { return active_.load(std::memory_order_acquire); }
    [[nodiscard]] std::string program() const;                          // what was detected (for the log)

private:
    std::atomic<bool> active_{false};
    std::atomic<bool> stop_{false};
    std::thread thread_;
    mutable std::mutex mtx_;
    std::string program_;
};

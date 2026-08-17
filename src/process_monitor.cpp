// src/process_monitor.cpp
#include "process_monitor.hpp"
#include <cstdlib>

bool ProcessMonitor::is_simulink_running() {
    const char* cmd = "pgrep -x \"[r]aspberrypi_get\" > /dev/null 2>&1";
    return (std::system(cmd) == 0);
}
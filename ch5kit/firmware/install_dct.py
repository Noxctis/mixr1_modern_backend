#!/usr/bin/env python3
"""Adds the earlier DCT estimator (discrete-time method) to your firmware so it can be run as  --estimator=dct --window=<us>.

Usage (from your firmware repo folder):   python3 ch5kit/firmware/install_dct.py
It edits  include/config.hpp  include/kinematics.hpp  src/kinematics.cpp  src/main.cpp  (or the same names in the current folder),
keeps their line endings, makes a .bak copy of each, and does nothing if it was already installed. If an anchor line is not
found it stops BEFORE writing anything and tells you which one."""
import os, shutil, sys

def find(name):
    for d in ("include", "src", "."):
        p = os.path.join(d, name)
        if os.path.exists(p):
            return p
    sys.exit(f"ERROR: cannot find {name}. Run this from your firmware repo folder (the one with include/ and src/).")

def load(p):
    b = open(p, "rb").read().decode("utf-8")
    crlf = "\r\n" in b
    return b.replace("\r\n", "\n"), crlf

def save(p, t, crlf):
    shutil.copy(p, p + ".bak")
    open(p, "wb").write((t.replace("\n", "\r\n") if crlf else t).encode("utf-8"))

def sub(t, old, new, name, count=1):
    if old not in t:
        sys.exit(f"ERROR: in {name} I did not find this line, so nothing was changed:\n    {old.strip()[:110]}\n"
                 "Send me that line of your file.")
    return t.replace(old, new, count)

cfg, hpp, cpp, mn = find("config.hpp"), find("kinematics.hpp"), find("kinematics.cpp"), find("main.cpp")
C, ccrlf = load(cfg); H, hcrlf = load(hpp); K, kcrlf = load(cpp); M, mcrlf = load(mn)
if "ESTIMATOR_DCT" in C:
    print("Already installed: nothing to do."); sys.exit(0)

# ---- config.hpp
C = sub(C, "inline bool ESTIMATOR_NEWEST = false;",
        "inline bool ESTIMATOR_DCT = false;                // --estimator=dct : the earlier discrete-time method (window = RPM_SAMPLE_WINDOW_US, set with --window=)\n    inline bool ESTIMATOR_NEWEST = false;",
        "config.hpp")

# ---- kinematics.hpp
H = sub(H, "    EncoderEdges prev_edges{};",
        "    EncoderEdges prev_edges{};\n"
        "    // State of estimator 3 (\"dct\"), the earlier discrete-time method\n"
        "    std::chrono::steady_clock::time_point dct_last_time{};\n"
        "    long long dct_last_count = 0;\n"
        "    long long dct_ticks = 0;\n"
        "    std::chrono::duration<double> dct_time{0.0};\n"
        "    double dct_rpm = 0.0;\n"
        "    bool dct_primed = false;", "kinematics.hpp")
H = sub(H, "    KinematicsState process_newest(const EncoderEdges& current, bool update_lcd);",
        "    KinematicsState process_newest(const EncoderEdges& current, bool update_lcd);\n"
        "    // Estimator 3 (\"dct\"), the earlier discrete-time method: count and clock are read every loop and accumulated until the elapsed\n"
        "    // time reaches RPM_SAMPLE_WINDOW_US; then rpm = |ticks / CPR| * 60 / time, held until the next window completes.\n"
        "    KinematicsState process_dct(long long current_count, int current_pwm);", "kinematics.hpp")

# ---- kinematics.cpp
K = sub(K, '#include "config.hpp"', '#include "config.hpp"\n#include <cmath>', "kinematics.cpp")
K = sub(K, "    first_run = true;\n}\n", "    first_run = true;\n    dct_primed = false;                         // the dct estimator re-primes itself from the next call\n    dct_ticks = 0;\n    dct_time = std::chrono::duration<double>::zero();\n    dct_rpm = 0.0;\n}\n", "kinematics.cpp")
K = K.rstrip("\n") + r"""

// Estimator 3 ("dct"): the earlier discrete-time method, kept so that it can be compared with the CET estimators on the same rig.
KinematicsState KinematicsEngine::process_dct(long long current_count, int current_pwm) {
    const auto now = std::chrono::steady_clock::now();
    if (!dct_primed) {
        dct_last_time = now;
        dct_last_count = current_count;
        dct_ticks = 0;
        dct_time = std::chrono::duration<double>::zero();
        dct_primed = true;
    }
    const std::chrono::duration<double> dt_sec = now - dct_last_time;
    dct_last_time = now;
    const long long delta_ticks = (current_count - dct_last_count) * Config::ENCODER_DIRECTION;
    dct_last_count = current_count;
    dct_ticks += delta_ticks;
    dct_time += dt_sec;

    bool refreshed = false;
    long long used_ticks = 0;
    uint32_t used_us = 0;
    if (dct_time.count() >= static_cast<double>(Config::RPM_SAMPLE_WINDOW_US) / 1000000.0) {
        used_ticks = dct_ticks;
        used_us = static_cast<uint32_t>(dct_time.count() * 1000000.0);
        if (current_pwm == 0 && std::abs(dct_ticks) <= Config::DEADBAND_TICK_THRESHOLD) {
            dct_rpm = 0.0;
            ema_filtered_rpm = 0.0;
        } else {
            dct_rpm = std::abs((static_cast<double>(dct_ticks) / Config::ENCODER_CPR) * (60.0 / dct_time.count()));
        }
        dct_ticks = 0;
        dct_time = std::chrono::duration<double>::zero();
        refreshed = true;
    }
    ema_filtered_rpm = (Config::RPM_ALPHA * dct_rpm) + ((1.0 - Config::RPM_ALPHA) * ema_filtered_rpm);

    KinematicsState out{dct_rpm, ema_filtered_rpm};
    out.new_snapshot = refreshed;
    out.delta_count = refreshed ? used_ticks : 0;
    out.delta_tick_us = refreshed ? used_us : 0;
    out.end_tick = 0;                                    // no edge time stamp is used by this method (age_us is not meaningful)
    return out;
}
"""

# ---- main.cpp
M = sub(M, '<< " estimator=" << (Config::ESTIMATOR_NEWEST ? "newest" : "aligned") << \'\\n\';',
        '<< " estimator=" << (Config::ESTIMATOR_DCT ? "dct" : (Config::ESTIMATOR_NEWEST ? "newest" : "aligned")) << " window_us=" << Config::RPM_SAMPLE_WINDOW_US << \'\\n\';', "main.cpp (t0 line)")
M = sub(M, "        auto state = Config::ESTIMATOR_NEWEST ? kinematics.process_newest(encoder.get_edge_snapshot(), false)\n"
           "                                              : kinematics.process(encoder.get_sync_snapshot(), current_pwm, false);",
        "        auto state = Config::ESTIMATOR_DCT ? kinematics.process_dct(encoder.get_count(), current_pwm)\n"
        "                   : Config::ESTIMATOR_NEWEST ? kinematics.process_newest(encoder.get_edge_snapshot(), false)\n"
        "                                              : kinematics.process(encoder.get_sync_snapshot(), current_pwm, false);", "main.cpp (test loop)")
M = sub(M, "        auto state = Config::ESTIMATOR_NEWEST ? kinematics.process_newest(encoder.get_edge_snapshot(), update_lcd)\n"
           "                                              : kinematics.process(snap, current_pwm, update_lcd);",
        "        auto state = Config::ESTIMATOR_DCT ? kinematics.process_dct(encoder.get_count(), current_pwm)\n"
        "                   : Config::ESTIMATOR_NEWEST ? kinematics.process_newest(encoder.get_edge_snapshot(), update_lcd)\n"
        "                                              : kinematics.process(snap, current_pwm, update_lcd);", "main.cpp (interactive loop)")
M = sub(M, '            Config::ESTIMATOR_NEWEST = (arg.substr(12) == "newest");\n'
           '            std::cout << "[CONFIG] Speed estimator: " << (Config::ESTIMATOR_NEWEST ? "newest-edge synchronous CET (paper)" : "aligned snapshots") << \'\\n\';',
        '            Config::ESTIMATOR_NEWEST = (arg.substr(12) == "newest");\n'
        '            Config::ESTIMATOR_DCT = (arg.substr(12) == "dct");\n'
        '            std::cout << "[CONFIG] Speed estimator: " << (Config::ESTIMATOR_DCT ? "dct (earlier discrete-time method)" : (Config::ESTIMATOR_NEWEST ? "newest-edge synchronous CET (paper)" : "aligned snapshots")) << \'\\n\';', "main.cpp (--estimator)")

save(cfg, C, ccrlf); save(hpp, H, hcrlf); save(cpp, K, kcrlf); save(mn, M, mcrlf)
print("Installed. Backups: config.hpp.bak kinematics.hpp.bak kinematics.cpp.bak main.cpp.bak\nNow rebuild (make). Run with  --estimator=dct --window=20000  (window in microseconds).")

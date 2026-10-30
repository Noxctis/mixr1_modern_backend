// src/main.cpp
#include <iostream>
#include <csignal>
#include <atomic>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
#include <limits>
#include <fstream>
#include <algorithm>
#include <numeric>
#include <string>
#include <vector>
#include <iomanip>
#include <sstream>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <fcntl.h>
#include <sys/file.h>

#include "config.hpp"
#include "process_monitor.hpp"
#include "kinematics.hpp"
#include "pi_controller.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "lcd.hpp"
#include "network.hpp"
#include "ec11.hpp"
#include "powermeter.hpp"
#include "torquesensor.hpp"
#include "modbus_torque.hpp"
#include "i2c_util.hpp"

std::atomic<bool> run_loop{true};

void signal_handler(int signum) {
    run_loop = false;
}

// ---- Zero (tare) the torque amplifier before a run -------------------------------------------------------------------------------------
// Waits until the shaft is at rest (encoder count unchanged for 1 s, at most 10 s), reads the offset, sends the amplifier's zero command,
// waits for its digital filter and checks that the reading really is ~0. Call it ONLY with no load on the sensor and BEFORE any other
// thread polls the same port. It never aborts the run: on a failure it prints a loud warning and carries on (the offset stays in the data).
template <class Sensor>
static bool zero_torque_at_rest(Sensor& sensor, AMT102Encoder& encoder, int settle_ms, double tol_nm, const char* tag) {
    const auto t0 = std::chrono::steady_clock::now();
    long long last_count = encoder.get_sync_snapshot().count;
    auto still_since = t0;
    bool at_rest = false;
    while (run_loop) {
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
        const auto tw = std::chrono::steady_clock::now();
        const long long c = encoder.get_sync_snapshot().count;
        if (c != last_count) { last_count = c; still_since = tw; }
        if (std::chrono::duration<double>(tw - still_since).count() >= 1.0) { at_rest = true; break; }
        if (std::chrono::duration<double>(tw - t0).count() >= 10.0) break;
    }
    if (!at_rest) {
        std::cout << tag << " WARNING: the shaft is not at rest: torque NOT zeroed (the zero must be taken at no load).\n";
        return false;
    }
    auto mean_read = [&](int n, double& out) -> bool {
        double sum = 0.0;
        int got = 0;
        for (int i = 0; i < n && run_loop; ++i) {
            double v = 0.0;
            if (sensor.read(v)) { sum += v; ++got; }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        if (got == 0) return false;
        out = sum / got;
        return true;
    };
    double before = std::nan("");
    mean_read(3, before);
    for (int attempt = 1; attempt <= 2 && run_loop; ++attempt) {
        if (!sensor.tare()) {
            std::cout << tag << " Torque zero command was not acknowledged (attempt " << attempt << " of 2).\n";
            continue;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(settle_ms));
        double after = std::nan("");
        if (!mean_read(5, after)) {
            std::cout << tag << " Torque zero sent, but the read-back failed (attempt " << attempt << " of 2).\n";
            continue;
        }
        if (std::fabs(after) <= tol_nm) {
            std::cout << tag << " Torque zeroed at no load: " << before << " -> " << after << " N.m\n";
            return true;
        }
        std::cout << tag << " Torque after the zero command is " << after << " N.m (more than +-" << tol_nm << "): attempt " << attempt << " of 2.\n";
    }
    std::cout << tag << " WARNING: torque was NOT zeroed. The reading keeps its offset (was " << before << " N.m): check the load, wiring and that nothing else uses the RS485 port.\n";
    return false;
}

struct TestOptions {
    bool fifo = false;
    bool use_pi = false;
    bool sweep = false;
    bool sine_mode = false;
    double sine_amplitude = 100.0;
    double sine_freq_hz = 0.1;
    double target_rpm = 460.0; 
    int fixed_pwm = 1000;
    double duration_sec = 30.0; 
    std::string csv_path = "timing_test.csv";
    bool stream = true;          // stream live to the dashboard / live_plot.py while the test runs
    bool wait_client = false;    // block at start until a dashboard connects (so the plot starts at t=0)
    bool ext_telemetry = false;  // 6-field packet (adds pwm/target/elapsed) for tools/live_plot.py
    double abort_rpm = 0.0;      // safety: stop the motor and end the run if the speed ever exceeds this (0 = off)
    double wait_rest_s = 0.0;    // before starting, wait (up to this many seconds) until the shaft has stopped turning (0 = don't wait)
    // open-loop two-level PWM step (system identification): hold step_from_pct until step_at_s, then step_to_pct
    bool step_mode = false;
    double step_from_pct = 0.0;
    double step_to_pct = 0.0;
    double step_at_s = 3.0;
    // Wattmeter (DFRobot SEN0291 / INA219) on I2C bus 1: logs bus voltage, current and power next to every sample. OFF unless you pass --power.
    bool power = false;              // OFF by default: enable with --power (test runs only; the normal daemon never touches the wattmeter)
    int power_addr = 0x45;
    int power_bus = -1;              // -1 = auto-detect (Config::I2C_BUS_POWER first, then the other bus); --power-bus=N forces one
    int power_div = 5;               // read the wattmeter every N-th loop (5 -> 20 Hz): keeps the control loop timing undisturbed
    double power_shunt_ohm = 0.010;
    double power_i_gain = 1.0;       // manual calibration against a multimeter: current_mA = gain * reading + offset
    double power_i_offset_ma = 0.0;
    double power_v_gain = 1.0;
    // Closed-loop REFERENCE STEP around the operating point: at ref_step_at seconds the target changes from target_rpm to ref_step_to.
    // A small step (for example 300 -> 400 RPM) stays well below the duty cap, so the response shows what the GAINS do (a start from rest
    // saturates the cap for about 1.3 s, which hides the gains). Metrics of the response are printed as rs_* in the [RESULT] line.
    double ref_step_to = 0.0;
    double ref_step_at = 15.0;
    double ref_step_to2 = 0.0;
    double ref_step_at2 = -1.0;
    double power_warn_w = -1.0;
    double max_duty_pct = 100.0; // hard cap on the PI output in % duty (water safety: the speed cannot exceed what this duty gives)
    bool start_at_zero = false;  // keep the motor off during the 250 ms preparation and switch it on exactly when the log starts (records the ramp from rest)
    // RS485 Modbus torque sensor (an inline strain-gauge amplifier, read with function code 0x03). OFF unless you pass --torque.
    // Register address, width, byte order and scale are amplifier-specific: set them from the manual, or find them with
    // tools/torque_calibration --probe. Mirrors --power in every way (same always-present-columns CSV pattern, same div cadence).
    bool torque = false;
    std::string torque_port = "/dev/ttyUSB0";
    int torque_baud = 9600;
    int torque_slave_id = 1;
    unsigned torque_reg_addr = 0;          // PLACEHOLDER: from the amplifier manual, or --torque-probe
    unsigned torque_reg_count = 2;         // 1 for int16/uint16, 2 for int32/uint32/float32
    std::string torque_format = "int32";   // int16 | uint16 | int32 | uint32 | float32
    std::string torque_word_order = "big";
    double torque_scale = 0.001;
    double torque_offset = 0.0;
    int torque_div = 5;                    // read the sensor every N-th loop (same idea as --power-div)
    bool tare = true;                      // zero the torque amplifier at no load before the run (--no-tare to skip)
    int tare_settle_ms = 1500;             // wait after the zero command so the amplifier's digital filter settles (--tare-settle-ms=N)
    int torque_stop_bits = 1;              // ATO-LCTR-OAR/DY610 factory default is N81 = 1 stop bit (set --torque-stopbits=2 if you changed it to N82)
};

bool set_fifo_priority() {
    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset); 
    pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);

    sched_param sch{};
    sch.sched_priority = 90;
    return pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) == 0;
}

bool parse_test_options(int argc, char** argv, TestOptions& options) {
    for (int i = 1; i < argc; ++i) {
        std::string argument = argv[i];
        if (argument == "--test") continue;
        if (argument.rfind("--cpr=", 0) == 0) continue;    
        if (argument.rfind("--window=", 0) == 0) continue; 
        
        if (argument == "--sweep") {
            options.sweep = true;
            options.sine_mode = false;
        } else if (argument == "--fixed") {
            options.sweep = false;
            options.sine_mode = false;
        } else if (argument == "--sine") {
            options.sine_mode = true;
            options.sweep = false;
        } else if (argument == "--fifo") {
            options.fifo = true;
        } else if (argument == "--no-fifo") {
            options.fifo = false;
        } else if (argument == "--pi") {
            options.use_pi = true;
        } else if (argument == "--no-pi") {
            options.use_pi = false;
        } else if (argument.rfind("--target=", 0) == 0) {
            options.target_rpm = std::stod(argument.substr(9));
        } else if (argument.rfind("--pwm=", 0) == 0) {
            options.fixed_pwm = std::stoi(argument.substr(6));
        } else if (argument.rfind("--duration=", 0) == 0) {
            options.duration_sec = std::stod(argument.substr(11));
        } else if (argument.rfind("--csv=", 0) == 0) {
            options.csv_path = argument.substr(6);
        } else if (argument.rfind("--sine-amp=", 0) == 0) {
            options.sine_amplitude = std::stod(argument.substr(11));
        } else if (argument.rfind("--sine-freq=", 0) == 0) {
            options.sine_freq_hz = std::stod(argument.substr(12));
        } else if (argument.rfind("--kp=", 0) == 0) {
            Config::GLOBAL_KP = std::stod(argument.substr(5));
        } else if (argument.rfind("--ki=", 0) == 0) {
            Config::GLOBAL_KI = std::stod(argument.substr(5));
        } else if (argument.rfind("--alpha=", 0) == 0) {
            Config::FEEDBACK_ALPHA = std::stod(argument.substr(8));
        } else if (argument == "--stream") {
            options.stream = true;
        } else if (argument == "--no-stream") {
            options.stream = false;
        } else if (argument == "--wait-client") {
            options.wait_client = true;
        } else if (argument.rfind("--step-from=", 0) == 0) {
            options.step_from_pct = std::stod(argument.substr(12));
            options.step_mode = true;
        } else if (argument.rfind("--step-to=", 0) == 0) {
            options.step_to_pct = std::stod(argument.substr(10));
            options.step_mode = true;
        } else if (argument.rfind("--step-at=", 0) == 0) {
            options.step_at_s = std::stod(argument.substr(10));
        } else if (argument == "--power") {
            options.power = true;
        } else if (argument == "--no-power") {
            options.power = false;
        } else if (argument.rfind("--power-bus=", 0) == 0) {
            options.power_bus = std::stoi(argument.substr(12));
        } else if (argument.rfind("--power-addr=", 0) == 0) {
            options.power_addr = std::stoi(argument.substr(13), nullptr, 0);          // accepts 0x45 or 69
        } else if (argument.rfind("--power-div=", 0) == 0) {
            options.power_div = std::max(1, std::stoi(argument.substr(12)));
        } else if (argument.rfind("--power-shunt-ohm=", 0) == 0) {
            options.power_shunt_ohm = std::stod(argument.substr(18));
        } else if (argument.rfind("--power-i-gain=", 0) == 0) {
            options.power_i_gain = std::stod(argument.substr(15));
        } else if (argument.rfind("--power-i-offset-ma=", 0) == 0) {
            options.power_i_offset_ma = std::stod(argument.substr(20));
        } else if (argument.rfind("--power-v-gain=", 0) == 0) {
            options.power_v_gain = std::stod(argument.substr(15));
        } else if (argument.rfind("--ref-step-to=", 0) == 0) {
            options.ref_step_to = std::stod(argument.substr(14));
        } else if (argument.rfind("--ref-step-at=", 0) == 0) {
            options.ref_step_at = std::stod(argument.substr(14));
        } else if (argument.rfind("--ref-step-to2=", 0) == 0) {
            options.ref_step_to2 = std::stod(argument.substr(15));
        } else if (argument.rfind("--ref-step-at2=", 0) == 0) {
            options.ref_step_at2 = std::stod(argument.substr(15));
        } else if (argument.rfind("--power-warn-w=", 0) == 0) {
            options.power_warn_w = std::stod(argument.substr(15));
        } else if (argument.rfind("--max-duty=", 0) == 0) {
            options.max_duty_pct = std::stod(argument.substr(11));
        } else if (argument == "--start-at-zero") {
            options.start_at_zero = true;
        } else if (argument.rfind("--wait-rest=", 0) == 0) {
            options.wait_rest_s = std::stod(argument.substr(12));
        } else if (argument.rfind("--abort-rpm=", 0) == 0) {
            options.abort_rpm = std::stod(argument.substr(12));
        } else if (argument == "--ext") {
            options.ext_telemetry = true;
        } else if (argument == "--torque") {
            options.torque = true;
        } else if (argument == "--no-torque") {
            options.torque = false;
        } else if (argument.rfind("--torque-port=", 0) == 0) {
            options.torque_port = argument.substr(14);
        } else if (argument.rfind("--torque-baud=", 0) == 0) {
            options.torque_baud = std::stoi(argument.substr(14));
        } else if (argument.rfind("--torque-slave=", 0) == 0) {
            options.torque_slave_id = std::stoi(argument.substr(15));
        } else if (argument.rfind("--torque-reg=", 0) == 0) {
            options.torque_reg_addr = static_cast<unsigned>(std::stoul(argument.substr(13)));
        } else if (argument.rfind("--torque-regcount=", 0) == 0) {
            options.torque_reg_count = static_cast<unsigned>(std::stoul(argument.substr(18)));
        } else if (argument.rfind("--torque-format=", 0) == 0) {
            options.torque_format = argument.substr(16);
        } else if (argument.rfind("--torque-word-order=", 0) == 0) {
            options.torque_word_order = argument.substr(20);
        } else if (argument.rfind("--torque-scale=", 0) == 0) {
            options.torque_scale = std::stod(argument.substr(15));
        } else if (argument.rfind("--torque-offset=", 0) == 0) {
            options.torque_offset = std::stod(argument.substr(16));
        } else if (argument.rfind("--torque-div=", 0) == 0) {
            options.torque_div = std::max(1, std::stoi(argument.substr(13)));
        } else if (argument.rfind("--torque-stopbits=", 0) == 0) {
            options.torque_stop_bits = std::stoi(argument.substr(18));
        } else if (argument == "--no-tare") {
            options.tare = false;
        } else if (argument == "--tare") {
            options.tare = true;
        } else if (argument.rfind("--tare-settle-ms=", 0) == 0) {
            options.tare_settle_ms = std::max(200, std::stoi(argument.substr(17)));
        } else {
            return false;
        }
    }
    if (options.step_mode) {                    // an open-loop step is neither PI, sweep nor sine
        options.use_pi = false;
        options.sweep = false;
        options.sine_mode = false;
        if (options.step_from_pct < 0.0 || options.step_from_pct > 100.0 || options.step_to_pct < 0.0 || options.step_to_pct > 100.0 ||
            options.step_at_s <= 0.0 || options.step_at_s >= options.duration_sec) return false;
    }
    if (options.ref_step_to2 > 0.0 && (options.ref_step_to <= 0.0 || options.ref_step_at2 <= options.ref_step_at || options.ref_step_at2 >= options.duration_sec - 1.0)) {
        std::cerr << "--ref-step-to2 requires --ref-step-to; --ref-step-at2 must be after step1 and before (duration - 1 s)\n";
        return false;
    }
    if (options.ref_step_to > 0.0 && !(options.use_pi && !options.sweep && !options.sine_mode && options.ref_step_at > 1.0 && options.ref_step_at < options.duration_sec - 3.0)) {
        std::cerr << "--ref-step-to needs closed-loop (PI) mode with a fixed target, and --ref-step-at between 1 s and (duration - 3 s)\n";
        return false;
    }
    return options.duration_sec > 0.0 && options.target_rpm >= 0.0 && options.max_duty_pct > 0.0 && options.max_duty_pct <= 100.0 &&
           options.fixed_pwm >= 0 && options.fixed_pwm <= 4095;
}

int run_test(const TestOptions& options) {
    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) return 1;

    bool fifo_active = false;
    if (options.fifo) {
        fifo_active = set_fifo_priority();
        if (!fifo_active) std::cerr << "[TEST] SCHED_FIFO request failed; continuing without it.\n";
    }

    AMT102Encoder encoder(pi, Config::PIN_ENC_A, Config::PIN_ENC_B, Config::PIN_ENC_X);
    MotorController motor(pi);
    KinematicsEngine kinematics;
    PIController controller;
    std::unique_ptr<PowerMeter> meter;
    if (options.power) {
        const int power_bus = (options.power_bus >= 0)
            ? options.power_bus
            : find_i2c_bus(pi, options.power_addr, Config::I2C_BUS_POWER, {Config::I2C_BUS_HARDWARE, Config::I2C_BUS_EXTRA});
        if (power_bus >= 0) meter = std::make_unique<PowerMeter>(pi, power_bus, options.power_addr, options.power_shunt_ohm, options.power_i_gain, options.power_i_offset_ma, options.power_v_gain);
        if (meter && meter->available()) {
            std::cout << "[TEST] Wattmeter (INA219) found at 0x" << std::hex << options.power_addr << std::dec << " on /dev/i2c-" << power_bus << ": logging bus_v, current_ma, power_w\n";
        } else {
            std::cout << "[TEST] No wattmeter answered at 0x" << std::hex << options.power_addr << std::dec;
            if (options.power_bus >= 0) std::cout << " on /dev/i2c-" << options.power_bus;
            else std::cout << " on /dev/i2c-" << Config::I2C_BUS_HARDWARE << " or /dev/i2c-" << Config::I2C_BUS_EXTRA << " (check the wiring, the pull-ups and i2cdetect -y <bus>)";
            std::cout << ": continuing without power columns\n";
            meter.reset();
        }
    }
    const char* power_status = options.power ? "absent" : "disabled";
    if (meter) power_status = "ok";
    long power_ok_reads = 0, power_fail_reads = 0;
    int power_fail_streak = 0;
    bool power_dropout = false;

    std::unique_ptr<TorqueSensor> tsensor;
    if (options.torque) {
        TorqueSensor::Settings ts;
        ts.port = options.torque_port;
        ts.baud = options.torque_baud;
        ts.slave_id = options.torque_slave_id;
        ts.reg_addr = options.torque_reg_addr;
        ts.reg_count = options.torque_reg_count;
        ts.value_format = options.torque_format;
        ts.word_order = options.torque_word_order;
        ts.scale = options.torque_scale;
        ts.offset = options.torque_offset;
        ts.stop_bits = options.torque_stop_bits;
        tsensor = std::make_unique<TorqueSensor>(ts);
        if (tsensor->available()) {
            std::cout << "[TEST] Torque sensor answering on " << options.torque_port << " (slave " << options.torque_slave_id
                      << ", reg " << options.torque_reg_addr << " x" << options.torque_reg_count << ", " << options.torque_format
                      << "): logging torque_nm\n";
        } else {
            std::cout << "[TEST] No torque sensor answered on " << options.torque_port << " (slave " << options.torque_slave_id
                      << "): check the port name, baud rate, slave ID, wiring and termination resistor. Continuing without the torque column.\n";
            tsensor.reset();
        }
    }
    const char* torque_status = options.torque ? "absent" : "disabled";
    if (tsensor) torque_status = "ok";
    long torque_ok_reads = 0, torque_fail_reads = 0;
    int torque_fail_streak = 0;
    bool torque_dropout = false;

    std::ofstream log(options.csv_path);
    if (!log) {
        std::cerr << "[TEST] Cannot open CSV: " << options.csv_path << '\n';
        motor.stop_motor();
        pigpio_stop(pi);
        return 1;
    }

    const std::string intended_mode = options.use_pi ? "PI" : "OpenLoop";
    const std::string intended_fifo = options.fifo ? "FIFO" : "NoFIFO";
    const std::string condition = intended_mode + "_" + intended_fifo;

    log << "elapsed_s,step_index,pwm_percent,loop_period_us,late_us,raw_rpm,filtered_rpm,target_rpm,pwm,error_rpm,intended_mode,intended_fifo,fifo_active,condition,fb_rpm";
    log << ",bus_v,current_ma,power_w,power_event";         // power_event = hi_w when power >= --power-warn-w
    log << ",torque_nm";                                    // always present (empty when there is no torque sensor), same convention as the power columns
    log << '\n';
    
    // ---- Live streaming (same TCP port/protocol as normal mode) ----
    TelemetryServer net;
    bool streaming = false;
    if (options.stream) {
        streaming = net.start_server(Config::TCP_PORT);
        if (!streaming) {
            std::cerr << "[TEST] Could not open port " << Config::TCP_PORT
                      << " (is the daemon already running?). Continuing without live streaming.\n";
        } else {
            std::cout << "[TEST] Live streaming on port " << Config::TCP_PORT
                      << (options.ext_telemetry ? " (extended packet)" : " (dashboard-compatible packet)")
                      << ". KP/KI/ALPHA commands apply live; RPM/PWM commands are ignored in test mode.\n";
            if (options.wait_client) {
                if (!net.wait_for_client()) {            // Ctrl+C while waiting
                    motor.stop_motor();
                    pigpio_stop(pi);
                    return 0;
                }
                std::cout << "[TEST] Dashboard connected. Starting test.\n";
            }
        }
    }

    // Make every run start from rest: the motor is still off here, so just wait until the encoder has not moved for 1 s.
    if (options.wait_rest_s > 0.0) {
        const auto t_wait0 = std::chrono::steady_clock::now();
        long long last_count = encoder.get_sync_snapshot().count;
        auto still_since = t_wait0;
        bool at_rest = false;
        double waited = 0.0;
        while (run_loop) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            const auto tw = std::chrono::steady_clock::now();
            const long long c = encoder.get_sync_snapshot().count;
            if (c != last_count) { last_count = c; still_since = tw; }
            waited = std::chrono::duration<double>(tw - t_wait0).count();
            if (std::chrono::duration<double>(tw - still_since).count() >= 1.0) { at_rest = true; break; }
            if (waited >= options.wait_rest_s) break;
        }
        std::cout << "[TEST] " << (at_rest ? "shaft at rest" : "WARNING: shaft still turning after waiting") << " (waited " << waited << " s)\n";
    }

    // Zero the torque amplifier now: the motor is still off here (the shaft is at rest; the helper checks the encoder itself).
    if (tsensor && options.tare) zero_torque_at_rest(*tsensor, encoder, options.tare_settle_ms, 2.0 * options.torque_scale, "[TEST]");

    const int step_from_counts = static_cast<int>(std::lround(options.step_from_pct / 100.0 * 4095.0));
    const int step_to_counts = static_cast<int>(std::lround(options.step_to_pct / 100.0 * 4095.0));
    int current_pwm = options.step_mode ? step_from_counts : (options.use_pi ? 0 : (options.sweep ? 0 : options.fixed_pwm));
    // PI fixed mode = step response from rest to target_rpm (sweep/sine override this per-iteration).
    double current_target = options.target_rpm;
    motor.set_pwm(options.start_at_zero ? 0 : current_pwm);

    // Idle draw of the driver board with the motor OFF (only when the motor really is off here); also gives the first row a valid reading.
    double idle_v = std::nan(""), idle_ma = std::nan("");
    if (meter && (options.use_pi || options.start_at_zero || options.sweep)) {
        double sv = 0.0, sa = 0.0;
        int got = 0;
        for (int k = 0; k < 5; ++k) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            double v = 0.0, ma = 0.0;
            if (meter->read(v, ma)) { sv += v; sa += ma; ++got; }
        }
        if (got > 0) { idle_v = sv / got; idle_ma = sa / got; }
    } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }

    kinematics.reset(encoder.get_sync_snapshot());
    controller.reset();
    controller.set_max_pwm(options.max_duty_pct / 100.0 * 4095.0);
    if (options.start_at_zero) motor.set_pwm(current_pwm);          // the motor starts exactly at t = 0 of the log

    const auto start = std::chrono::steady_clock::now();
    auto next_wake = start;
    auto previous_tick = start;
    int step_index = -1;
    std::vector<double> periods_us;
    std::vector<double> late_us;
    std::vector<double> rpm_samples;
    std::vector<double> errors;
    std::vector<double> pwm_samples;
    std::vector<double> time_samples;
    std::vector<double> volt_samples;
    std::vector<double> cur_samples;
    std::vector<double> torque_samples;
    double last_v = idle_v, last_ma = idle_ma;
    if (meter && !(last_v == last_v)) {                      // no idle reading: take one now so the first rows are valid
        double v = 0.0, ma = 0.0;
        if (meter->read(v, ma)) { last_v = v; last_ma = ma; }
    }
    double last_torque = std::nan("");
    if (tsensor) {
        double t0v = 0.0;
        if (tsensor->read(t0v)) last_torque = t0v;            // first row gets a real value where possible, same idea as the wattmeter's idle read
    }

    const double total_duration = options.sweep ? options.duration_sec * 11.0 : options.duration_sec;
    
    if (options.sine_mode) {
        std::cout << "[TEST] Sine Wave Mode | Target: " << options.target_rpm << " RPM +/- " 
                  << options.sine_amplitude << " | Freq: " << options.sine_freq_hz << " Hz\n";
    }

    bool aborted = false;
    double abort_peak_rpm = 0.0, abort_time_s = 0.0;
    while (run_loop) {
        next_wake += std::chrono::microseconds(Config::LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed >= total_duration) break;

        if (options.sine_mode && options.use_pi) {
            current_target = options.target_rpm + (options.sine_amplitude * std::sin(2.0 * M_PI * options.sine_freq_hz * elapsed));
        } else if (options.sine_mode && !options.use_pi) {
            // FIX: Directly modulate PWM for open-loop sine wave
            current_pwm = options.fixed_pwm + static_cast<int>(options.sine_amplitude * std::sin(2.0 * M_PI * options.sine_freq_hz * elapsed));
            current_pwm = std::clamp(current_pwm, 0, 4095);
            motor.set_pwm(current_pwm);
        } else if (options.step_mode) {
            const int want = (elapsed < options.step_at_s) ? step_from_counts : step_to_counts;
            if (want != current_pwm) {
                current_pwm = want;
                motor.set_pwm(current_pwm);
                std::cout << "[TEST] PWM step " << options.step_from_pct << "% -> " << options.step_to_pct << "% at t=" << elapsed << " s\n";
            }
        } else if (options.sweep) {
            const int new_step_index = std::min(10, static_cast<int>(elapsed / options.duration_sec));
            if (new_step_index != step_index) {
                step_index = new_step_index;
                if (options.use_pi) {
                    current_target = options.target_rpm * step_index / 10.0;
                    controller.reset();
                } else {
                    current_pwm = (step_index * 10 * 4095) / 100;
                    motor.set_pwm(current_pwm);
                }
                std::cout << "[TEST] " << (options.use_pi ? "RPM target " : "PWM step ")
                          << (options.use_pi ? current_target : step_index * 10)
                          << (options.use_pi ? " RPM" : "%") << " for "
                          << options.duration_sec << " seconds\n";
            }
        }

        const double period = std::chrono::duration<double, std::micro>(now - previous_tick).count();
        const double lateness = std::max(0.0, std::chrono::duration<double, std::micro>(now - next_wake).count());
        previous_tick = now;
        
        // Accept live KP/KI/ALPHA (and CPR/WIN) from the dashboard; RPM/PWM commands are dummies here.
        if (streaming) {
            net.poll_for_client();
            if (net.has_client()) {
                double ignored_rpm = 0.0; int ignored_pwm = 0; bool ignored_mode = false;
                net.receive_command(ignored_rpm, ignored_pwm, ignored_mode);
            }
        }

        auto state = kinematics.process(encoder.get_sync_snapshot(), current_pwm, false);
        if (options.abort_rpm > 0.0 && state.exact_rpm > options.abort_rpm) {      // safety cut-off
            motor.set_pwm(0);
            aborted = true;
            abort_peak_rpm = state.exact_rpm;
            abort_time_s = elapsed;
            std::cout << "[TEST] ABORT: speed " << state.exact_rpm << " RPM exceeded the " << options.abort_rpm
                      << " RPM limit at t=" << elapsed << " s - motor stopped\n";
            break;
        }

        if (options.use_pi && options.ref_step_to > 0.0 && elapsed >= options.ref_step_at) current_target = options.ref_step_to;
        if (options.use_pi && options.ref_step_to2 > 0.0 && elapsed >= options.ref_step_at2) current_target = options.ref_step_to2;
        if (options.use_pi) {
            current_pwm = controller.compute(current_target, state.exact_rpm, period / 1000000.0);
            motor.set_pwm(current_pwm);
        }

        // The value the controller is really acting on (PI) or the display EMA (open loop).
        const double fb_rpm = options.use_pi ? controller.feedback_rpm(state.exact_rpm) : state.ema_filtered_rpm;

        if (streaming && net.has_client()) {
            if (options.ext_telemetry || net.wants_ext()) {
                PowerSample ps;
                if (meter && std::isfinite(last_v) && std::isfinite(last_ma)) { ps.bus_v = last_v; ps.current_ma = last_ma; ps.power_w = last_v * last_ma / 1000.0; }
                net.send_packet_ext(state.exact_rpm, fb_rpm, encoder.get_revolutions(),
                                    current_pwm * 100.0 / 4095.0, current_target, elapsed, ps, last_torque);
            } else {
                net.send_packet(state.exact_rpm, fb_rpm, encoder.get_revolutions());
            }
        }

        if (meter && (time_samples.size() % static_cast<size_t>(options.power_div)) == 0) {
            double v = 0.0, ma = 0.0;
            if (meter->read(v, ma)) {
                last_v = v; last_ma = ma; ++power_ok_reads; power_fail_streak = 0;
            } else {
                ++power_fail_reads;
                if (++power_fail_streak > 10) { last_v = std::nan(""); last_ma = std::nan(""); power_dropout = true; }   // do not keep writing a stale value
            }
        }

        if (tsensor && (time_samples.size() % static_cast<size_t>(options.torque_div)) == 0) {
            double tq = 0.0;
            if (tsensor->read(tq)) {
                last_torque = tq; ++torque_ok_reads; torque_fail_streak = 0;
            } else {
                ++torque_fail_reads;
                if (++torque_fail_streak > 10) { last_torque = std::nan(""); torque_dropout = true; }   // RS485 dropout: stop writing a stale value, same rule as the wattmeter
            }
        }

        const double error = current_target - state.exact_rpm;
        const int pwm_percent = options.sweep ? step_index * 10 : (current_pwm * 100) / 4095;
        log << std::fixed << std::setprecision(6) << elapsed << ',' << step_index << ',' << pwm_percent << ','
            << period << ',' << lateness << ','
            << state.exact_rpm << ',' << state.ema_filtered_rpm << ',' << current_target << ','
            << current_pwm << ',' << error << ','
            << intended_mode << ',' << intended_fifo << ',' << (fifo_active ? "true" : "false") << ',' << condition << ','
            << fb_rpm;
        const double pw_now = (meter && std::isfinite(last_v) && std::isfinite(last_ma)) ? (last_v * last_ma / 1000.0) : std::nan("");
        if (std::isfinite(pw_now)) log << ',' << last_v << ',' << last_ma << ',' << pw_now;
        else log << ",,,";
        log << ',' << ((options.power_warn_w > 0.0 && std::isfinite(pw_now) && pw_now >= options.power_warn_w) ? "hi_w" : "");
        if (std::isfinite(last_torque)) log << ',' << last_torque;
        else log << ',';
        log << '\n';
        periods_us.push_back(period);
        late_us.push_back(lateness);
        rpm_samples.push_back(state.exact_rpm);
        errors.push_back(std::abs(error));
        pwm_samples.push_back(static_cast<double>(current_pwm));
        time_samples.push_back(elapsed);
        volt_samples.push_back(last_v);
        cur_samples.push_back(last_ma);
        torque_samples.push_back(last_torque);
    }

    motor.stop_motor();
    pigpio_stop(pi);
    if (aborted) {
        std::cout << std::fixed << std::setprecision(3) << "[ABORTED] kp=" << Config::GLOBAL_KP << " ki=" << Config::GLOBAL_KI
                  << " alpha=" << Config::FEEDBACK_ALPHA << " target=" << options.target_rpm << " peak_rpm=" << abort_peak_rpm
                  << " at_s=" << abort_time_s << '\n';
    }
    
    if (periods_us.empty()) return 1;

    const auto mean = [](const std::vector<double>& values) {
        return std::accumulate(values.begin(), values.end(), 0.0) / values.size();
    };
    const double period_mean = mean(periods_us);
    double variance = 0.0;
    for (double period : periods_us) variance += (period - period_mean) * (period - period_mean);
    variance /= periods_us.size();
    const auto max_late = *std::max_element(late_us.begin(), late_us.end());
    const auto late_cycles = std::count_if(late_us.begin(), late_us.end(), [](double value) { return value > 0.0; });

    std::cout << "[TEST] fifo=" << (fifo_active ? "active" : "off")
              << " pi=" << (options.use_pi ? "on" : "off")
              << " samples=" << periods_us.size()
              << " period_mean_us=" << period_mean
              << " period_std_us=" << std::sqrt(variance)
              << " max_late_us=" << max_late
              << " late_cycles=" << late_cycles
              << " mean_rpm=" << mean(rpm_samples)
              << " mean_abs_error_rpm=" << mean(errors) << '\n';
    std::cout << "[TEST] CSV saved to " << options.csv_path << '\n';

    // ---- Step-response / jitter metrics (PI, fixed target only) ----
    // Steady-state window = last 40% of the run. Parsed by tools/run_matrix.sh via the [RESULT] tag.
    if (options.use_pi && !options.sweep && !options.sine_mode && options.target_rpm > 0.0) {
        const size_t n_all = rpm_samples.size();
        const bool ref_step = options.ref_step_to > 0.0;
        size_t n = n_all;                                   // with a reference step, the startup / steady-state metrics use only the part BEFORE the step
        if (ref_step) { n = 0; while (n < n_all && time_samples[n] < options.ref_step_at) ++n; }
        const size_t ss = static_cast<size_t>(static_cast<double>(n) * 0.6);
        const size_t cnt = n - ss;
        if (cnt > 2) {
            double rpm_mean = 0.0, pwm_mean = 0.0;
            for (size_t i = ss; i < n; ++i) { rpm_mean += rpm_samples[i]; pwm_mean += pwm_samples[i]; }
            rpm_mean /= static_cast<double>(cnt);
            pwm_mean /= static_cast<double>(cnt);

            double rpm_var = 0.0, pwm_var = 0.0, diff_sq = 0.0;
            for (size_t i = ss; i < n; ++i) {
                rpm_var += (rpm_samples[i] - rpm_mean) * (rpm_samples[i] - rpm_mean);
                pwm_var += (pwm_samples[i] - pwm_mean) * (pwm_samples[i] - pwm_mean);
                if (i > ss) {
                    const double d = pwm_samples[i] - pwm_samples[i - 1];
                    diff_sq += d * d;
                }
            }
            const double rpm_std = std::sqrt(rpm_var / static_cast<double>(cnt));
            const double to_pct = 100.0 / 4095.0;
            const double pwm_std_pct = std::sqrt(pwm_var / static_cast<double>(cnt)) * to_pct;
            const double pwm_jitter_pct = std::sqrt(diff_sq / static_cast<double>(cnt - 1)) * to_pct;

            const double max_rpm = *std::max_element(rpm_samples.begin(), rpm_samples.end());
            const double overshoot_pct = std::max(0.0, (max_rpm - options.target_rpm) / options.target_rpm * 100.0);

            // Settling time: last moment raw RPM was outside +/-5% of target (-1 = never settled).
            const double band = 0.05 * options.target_rpm;
            double settle_s = 0.0;
            for (size_t i = n; i-- > 0;) {
                if (std::abs(rpm_samples[i] - options.target_rpm) > band) {
                    settle_s = (i == n - 1) ? -1.0 : time_samples[i];
                    break;
                }
            }

            // Electrical measurements over the same steady-state window. Gaps (no wattmeter, or a dropout) are skipped, never replaced by zeros.
            std::ostringstream pw;
            {
                const char* st = power_dropout ? "dropout" : power_status;
                pw << " power_status=" << st;
                if (options.power) pw << " power_reads_ok=" << power_ok_reads << " power_reads_fail=" << power_fail_reads;
                if (meter && volt_samples.size() == n_all) {
                    double vm = 0.0, im = 0.0;
                    size_t k = 0;
                    for (size_t i = ss; i < n; ++i) if (std::isfinite(volt_samples[i]) && std::isfinite(cur_samples[i])) { vm += volt_samples[i]; im += cur_samples[i]; ++k; }
                    if (k > 2) {
                        vm /= static_cast<double>(k);
                        im /= static_cast<double>(k);
                        double iv = 0.0;
                        for (size_t i = ss; i < n; ++i) if (std::isfinite(cur_samples[i])) iv += (cur_samples[i] - im) * (cur_samples[i] - im);
                        iv = std::sqrt(iv / static_cast<double>(k));
                        pw << std::fixed << std::setprecision(4) << " v_mean_V=" << vm << " i_mean_mA=" << im << " i_std_mA=" << iv << " p_mean_W=" << (vm * im / 1000.0)
                           << " p_idle_W=" << ((idle_v == idle_v && idle_ma == idle_ma) ? idle_v * idle_ma / 1000.0 : -1.0);
                    }
                }
            }

            // Rise time 10-90 % of the target (first crossing of a 5-sample moving average; -1 = never reached) and settling to +/-2 %.
            auto movavg = [&](size_t i) {
                const size_t lo = i >= 2 ? i - 2 : 0, hi = std::min(n - 1, i + 2);
                double acc = 0.0;
                for (size_t k = lo; k <= hi; ++k) acc += rpm_samples[k];
                return acc / static_cast<double>(hi - lo + 1);
            };
            double t10 = -1.0, t90 = -1.0;
            for (size_t i = 0; i < n; ++i) {
                const double m = movavg(i);
                if (t10 < 0.0 && m >= 0.1 * options.target_rpm) t10 = time_samples[i];
                if (t90 < 0.0 && m >= 0.9 * options.target_rpm) { t90 = time_samples[i]; break; }
            }
            const double rise_s = (t10 >= 0.0 && t90 >= 0.0) ? (t90 - t10) : -1.0;
            const double band2 = 0.02 * options.target_rpm;
            double settle2_s = 0.0;
            for (size_t i = n; i-- > 0;) {
                if (std::abs(rpm_samples[i] - options.target_rpm) > band2) {
                    settle2_s = (i == n - 1) ? -1.0 : time_samples[i];
                    break;
                }
            }

            // ---- closed-loop reference step: metrics of the response AFTER the step (5-sample moving average) ----
            std::ostringstream rs;
            if (ref_step && n < n_all) {
                auto ma5 = [&](size_t i) {
                    const size_t lo = i >= 2 ? i - 2 : 0, hi = std::min(n_all - 1, i + 2);
                    double acc = 0.0;
                    for (size_t k = lo; k <= hi; ++k) acc += rpm_samples[k];
                    return acc / static_cast<double>(hi - lo + 1);
                };
                const double y_pre = rpm_mean;                                  // steady state just before the step
                const size_t m0 = n;                                             // first sample after the step
                const size_t m = n_all - m0;
                const size_t fs = m0 + static_cast<size_t>(0.7 * static_cast<double>(m));   // final window = last 30 % of the post-step part
                double y_fin = 0.0;
                for (size_t i = fs; i < n_all; ++i) y_fin += rpm_samples[i];
                y_fin /= static_cast<double>(n_all - fs);
                const double delta = y_fin - y_pre;
                if (m > 20 && std::abs(delta) > 5.0) {
                    const double sgn = delta > 0.0 ? 1.0 : -1.0;
                    double peak = y_fin;
                    for (size_t i = m0; i < n_all; ++i) { const double v = ma5(i); if ((v - y_fin) * sgn > (peak - y_fin) * sgn) peak = v; }
                    const double rs_os = std::max(0.0, (peak - y_fin) * sgn / std::abs(delta) * 100.0);
                    double u10 = -1.0, u90 = -1.0;
                    for (size_t i = m0; i < n_all; ++i) {
                        const double f = (ma5(i) - y_pre) * sgn / std::abs(delta);
                        if (u10 < 0.0 && f >= 0.1) u10 = time_samples[i];
                        if (u90 < 0.0 && f >= 0.9) { u90 = time_samples[i]; break; }
                    }
                    const double band_rs = std::max(0.05 * std::abs(delta), 3.0);        // +-5 % of the step, but never tighter than 3 RPM (the speed noise)
                    double rs_settle = 0.0;
                    for (size_t i = n_all; i-- > m0;) {
                        if (std::abs(ma5(i) - y_fin) > band_rs) { rs_settle = (i == n_all - 1) ? -1.0 : time_samples[i] - options.ref_step_at; break; }
                    }
                    const double cap_counts = options.max_duty_pct / 100.0 * 4095.0;     // did the step hit the duty cap? then it measures the cap, not the gains
                    double peak_counts = 0.0, sat_s = 0.0;
                    for (size_t i = m0; i < n_all; ++i) {
                        peak_counts = std::max(peak_counts, pwm_samples[i]);
                        if (pwm_samples[i] >= cap_counts - 2.0) sat_s += static_cast<double>(Config::LOOP_DELAY_US) * 1.0e-6;
                    }
                    double fr_var = 0.0, fp_mean = 0.0, fd_sq = 0.0;
                    for (size_t i = fs; i < n_all; ++i) { fr_var += (rpm_samples[i] - y_fin) * (rpm_samples[i] - y_fin); fp_mean += pwm_samples[i]; }
                    fp_mean /= static_cast<double>(n_all - fs);
                    for (size_t i = fs + 1; i < n_all; ++i) { const double d2 = pwm_samples[i] - pwm_samples[i - 1]; fd_sq += d2 * d2; }
                    rs << std::fixed << std::setprecision(3)
                       << " rs_from_rpm=" << y_pre << " rs_to_rpm=" << y_fin
                       << " rs_overshoot_pct=" << rs_os
                       << " rs_rise_s=" << ((u10 >= 0.0 && u90 >= 0.0) ? (u90 - u10) : -1.0)
                       << " rs_settle_s=" << rs_settle << " rs_band_rpm=" << band_rs
                       << " rs_peak_duty_pct=" << (peak_counts * to_pct) << " rs_sat_s=" << sat_s
                       << " rs_rpm_std=" << std::sqrt(fr_var / static_cast<double>(n_all - fs))
                       << " rs_pwm_jitter_pct=" << (std::sqrt(fd_sq / static_cast<double>(n_all - fs - 1)) * to_pct);
                }
            }

            // Torque measurements over the same steady-state window, same gap-handling rule as the electrical block above.
            std::ostringstream tq;
            {
                const char* st = torque_dropout ? "dropout" : torque_status;
                tq << " torque_status=" << st;
                if (options.torque) tq << " torque_reads_ok=" << torque_ok_reads << " torque_reads_fail=" << torque_fail_reads;
                if (tsensor && torque_samples.size() == n_all) {
                    double tm = 0.0;
                    size_t k = 0;
                    for (size_t i = ss; i < n; ++i) if (std::isfinite(torque_samples[i])) { tm += torque_samples[i]; ++k; }
                    if (k > 2) {
                        tm /= static_cast<double>(k);
                        double tv_ = 0.0;
                        for (size_t i = ss; i < n; ++i) if (std::isfinite(torque_samples[i])) tv_ += (torque_samples[i] - tm) * (torque_samples[i] - tm);
                        tv_ = std::sqrt(tv_ / static_cast<double>(k));
                        tq << std::fixed << std::setprecision(5) << " torque_mean_Nm=" << tm << " torque_std_Nm=" << tv_;
                    }
                }
            }

            std::cout << std::fixed << std::setprecision(3)
                      << "[RESULT] kp=" << Config::GLOBAL_KP
                      << " ki=" << Config::GLOBAL_KI
                      << " alpha=" << Config::FEEDBACK_ALPHA
                      << " target=" << options.target_rpm
                      << " rpm_mean=" << rpm_mean
                      << " rpm_std=" << rpm_std
                      << " sse=" << (options.target_rpm - rpm_mean)
                      << " pwm_mean_pct=" << (pwm_mean * to_pct)
                      << " pwm_std_pct=" << pwm_std_pct
                      << " pwm_jitter_pct=" << pwm_jitter_pct
                      << " overshoot_pct=" << overshoot_pct
                      << " settle5_s=" << settle_s
                      << " rise10_90_s=" << rise_s
                      << " settle2_s=" << settle2_s
                      << rs.str()
                      << pw.str()
                      << tq.str()
                      << " aborted=" << (aborted ? 1 : 0) << '\n';
        }
    }
    return 0;
}

// Only ONE MIXR-1 process may drive the motor at a time. A detached run_matrix.sh keeps running after the terminal is closed, and a second
// daemon started on top of it would fight it for the same pins. The lock is released automatically when the process exits or crashes.
static void acquire_single_instance_lock() {
    const int fd = open("/tmp/mixr1_daemon.lock", O_CREAT | O_RDWR, 0666);
    if (fd < 0) return;                                           // cannot create the lock file: do not block the run
    if (flock(fd, LOCK_EX | LOCK_NB) != 0) {
        std::cerr << "[CRITICAL] Another MIXR-1 process is already running and controlling the motor. NOT starting.\n"
                  << "  See what it is:  pgrep -a mixr1_daemon ; pgrep -af 'run_matrix|run_steps'\n"
                  << "  Stop it:         sudo ./motor_off.sh\n";
        std::exit(3);
    }
    // the descriptor stays open on purpose for the life of the process
}

// ---------------------------------------------------------------------------------------------------------------------------------
// ./mixr1_daemon --test-torque [--seconds=10] [--torque-port=/dev/ttyUSB0] [--torque-baud=9600] [--torque-slave=1]
//                [--torque-reg=0] [--torque-regcount=2] [--torque-format=int32] [--torque-word-order=big] [--torque-scale=0.001]
// Checks the RS485 Modbus torque amplifier on its own: does the port open, does the slave answer function 0x03 with a valid
// CRC, are the readings stable. It does NOT drive the motor. Exit code 0 = PASS, 1 = check the wiring / register settings.
// ---------------------------------------------------------------------------------------------------------------------------------
static int run_torque_test(int argc, char** argv) {
    TorqueSensor::Settings s;
    double seconds = 10.0;
    std::string csv_path = "torque_test.csv";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--seconds=", 0) == 0) seconds = std::max(2.0, std::stod(a.substr(10)));
        else if (a.rfind("--torque-port=", 0) == 0) s.port = a.substr(14);
        else if (a.rfind("--torque-baud=", 0) == 0) s.baud = std::stoi(a.substr(14));
        else if (a.rfind("--torque-slave=", 0) == 0) s.slave_id = std::stoi(a.substr(15));
        else if (a.rfind("--torque-reg=", 0) == 0) s.reg_addr = static_cast<unsigned>(std::stoul(a.substr(13)));
        else if (a.rfind("--torque-regcount=", 0) == 0) s.reg_count = static_cast<unsigned>(std::stoul(a.substr(18)));
        else if (a.rfind("--torque-format=", 0) == 0) s.value_format = a.substr(16);
        else if (a.rfind("--torque-word-order=", 0) == 0) s.word_order = a.substr(20);
        else if (a.rfind("--torque-scale=", 0) == 0) s.scale = std::stod(a.substr(15));
        else if (a.rfind("--torque-offset=", 0) == 0) s.offset = std::stod(a.substr(16));
        else if (a.rfind("--torque-stopbits=", 0) == 0) s.stop_bits = std::stoi(a.substr(18));
        else if (a.rfind("--csv=", 0) == 0) csv_path = a.substr(6);
    }
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);

    std::cout << "=== TORQUE SENSOR TEST (RS485 Modbus RTU, function 0x03) ===\n"
              << "The motor is NOT driven.\n\n"
              << "Step 1: opening " << s.port << " at " << s.baud << " baud, slave " << s.slave_id
              << ", register " << s.reg_addr << " x" << s.reg_count << " (" << s.value_format << ", "
              << s.word_order << "-endian, scale " << s.scale << ") ...\n";
    TorqueSensor sensor(s);
    if (!sensor.available()) {
        std::cout << "\nRESULT: FAIL - no valid response from slave " << s.slave_id << " on " << s.port << ".\n"
                  << "  * check the port name (ls /dev/ttyUSB* /dev/ttyACM*) and that nothing else has it open (sudo fuser -v " << s.port << ")\n"
                  << "  * check the baud rate, slave ID and register address against the amplifier's manual\n"
                  << "  * check A/B (D+/D-) wiring is not swapped, and that a 120 ohm termination resistor is fitted at each end of the bus\n"
                  << "  * try --torque-regcount=1 if the amplifier uses a 16-bit register instead of 32-bit\n";
        return 1;
    }
    std::cout << "yes - slave answered with a valid frame.\n";
    {   // ATO-LCTR-OAR/DY610 only: register 68 ("decimal point", 0-4) is what the amplifier was last set to via its own
        // keys. scale should be 10^-that. If this amplifier was never calibrated on-device with a real weight (SL/SH keys),
        // the raw register value is meaningless no matter what scale you apply here -- do that first.
        std::vector<uint16_t> dp;
        if (sensor.read_raw(68, 1, dp)) {
            std::cout << "\n(DY610 check) Register 68 (decimal point setting): " << dp[0]
                      << "  ->  scale should be 10^-" << dp[0] << " = " << std::pow(10.0, -static_cast<double>(dp[0]))
                      << "\nIf that doesn't match what you passed with --torque-scale, fix the flag. And if the amplifier hasn't been\n"
                      << "calibrated with a real weight using its SL (zero) and SH (span) keys yet, do that before trusting any reading.\n";
        }
    }
    std::cout << "\nStep 2: reading for " << seconds << " s (5 reads per second; one line every 0.5 s)\n"
              << "   t(s)    torque " << s.unit << "    raw registers\n";
    std::vector<double> tv, vals;
    long fails = 0;
    std::ofstream csv(csv_path);
    csv << "time_s,torque_" << s.unit << ",raw_registers,ok\n";
    const auto t0 = std::chrono::steady_clock::now();
    auto next = t0;
    int printed = 0;
    while (run_loop) {
        next += std::chrono::milliseconds(200);
        std::this_thread::sleep_until(next);
        const auto now = std::chrono::steady_clock::now();
        const double t = std::chrono::duration<double>(now - t0).count();
        if (t > seconds) break;
        double v = 0.0;
        std::vector<uint16_t> regs;
        const bool ok = sensor.read(v) && sensor.read_raw(s.reg_addr, s.reg_count, regs);
        if (ok) { tv.push_back(t); vals.push_back(v); } else { ++fails; }
        csv << std::fixed << std::setprecision(3) << t << ',';
        if (ok) csv << v; else csv << "";
        csv << ',';
        if (ok) { for (size_t i = 0; i < regs.size(); ++i) { if (i) csv << '|'; csv << regs[i]; } }
        csv << ',' << (ok ? 1 : 0) << '\n';
        if (t >= 0.5 * printed) {
            ++printed;
            if (ok) {
                std::cout << "  " << std::fixed << std::setprecision(2) << std::setw(5) << t << "  " << std::setw(12) << v << "    ";
                for (size_t i = 0; i < regs.size(); ++i) { if (i) std::cout << '|'; std::cout << regs[i]; }
                std::cout << '\n';
            } else {
                std::cout << "  " << std::fixed << std::setprecision(2) << std::setw(5) << t << "   READ FAILED\n";
            }
        }
    }
    csv.close();
    if (vals.size() < 5) { std::cout << "\nRESULT: FAIL - fewer than 5 successful reads (" << fails << " failed).\n"; return 1; }
    double mean = 0.0; for (double v : vals) mean += v; mean /= static_cast<double>(vals.size());
    double sd = 0.0; for (double v : vals) sd += (v - mean) * (v - mean); sd = std::sqrt(sd / static_cast<double>(vals.size()));
    const double mn = *std::min_element(vals.begin(), vals.end());
    const double mx = *std::max_element(vals.begin(), vals.end());
    std::cout << std::fixed << "\nSummary over " << vals.size() << " reads (" << fails << " failed):\n"
              << std::setprecision(5) << "   torque : mean " << mean << " " << s.unit << "   std " << sd
              << "   min " << mn << "   max " << mx << '\n' << "\nChecks:\n";
    bool pass = true;
    auto check = [&](bool ok, const std::string& good, const std::string& bad) {
        std::cout << "   [" << (ok ? " OK " : "FAIL") << "] " << (ok ? good : bad) << '\n';
        if (!ok) pass = false;
    };
    check(fails == 0, "no failed reads", std::to_string(fails) + " reads failed: a marginal bus, wrong baud, or a loose termination/ground");
    check(sd < 0.05 * (std::abs(mean) > 1e-9 ? std::abs(mean) : 1.0) || sd < 0.01,
          "readings are stable for an unloaded sensor", "readings are noisy: check grounding, cable routing away from the motor leads, and shielding");
    std::cout << "\n   With nothing hanging on the sensor, this is your ZERO reading: " << mean << " " << s.unit
              << " (use this as --torque-offset, or the calibration tool's zero convention, so a no-load reading reads as 0).\n"
              << "   Now run the calibration tool with known weights:  tools/torque_calibration --port=" << s.port << '\n'
              << "\nData written to " << csv_path << "\nRESULT: " << (pass ? "PASS" : "CHECK - see the FAIL lines above") << '\n';
    return pass ? 0 : 1;
}

// ---------------------------------------------------------------------------------------------------------------------------------
// ./mixr1_daemon --test-wattmeter [--seconds=10] [--power-addr=0x45] [--power-bus=N] [--csv=wattmeter_test.csv]
// Checks the DFRobot wattmeter (INA219) on its own: which bus answers, whether it behaves like an INA219, whether the readings are
// plausible and stable, how long a read takes. It does NOT drive the motor. Exit code 0 = PASS, 1 = check the wiring, 2 = no pigpiod.
// ---------------------------------------------------------------------------------------------------------------------------------
static int run_wattmeter_test(int argc, char** argv) {
    int addr = 0x45, forced_bus = -1;
    double seconds = 10.0, shunt = 0.010, i_gain = 1.0, i_off = 0.0, v_gain = 1.0;
    std::string csv_path = "wattmeter_test.csv";
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a.rfind("--seconds=", 0) == 0) seconds = std::max(2.0, std::stod(a.substr(10)));
        else if (a.rfind("--power-addr=", 0) == 0) addr = std::stoi(a.substr(13), nullptr, 0);
        else if (a.rfind("--power-bus=", 0) == 0) forced_bus = std::stoi(a.substr(12));
        else if (a.rfind("--power-shunt-ohm=", 0) == 0) shunt = std::stod(a.substr(18));
        else if (a.rfind("--power-i-gain=", 0) == 0) i_gain = std::stod(a.substr(15));
        else if (a.rfind("--power-i-offset-ma=", 0) == 0) i_off = std::stod(a.substr(20));
        else if (a.rfind("--power-v-gain=", 0) == 0) v_gain = std::stod(a.substr(15));
        else if (a.rfind("--csv=", 0) == 0) csv_path = a.substr(6);
    }
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    const int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) { std::cerr << "[CRITICAL] cannot connect to pigpiod (start it with: sudo pigpiod)\n"; return 2; }

    std::cout << "=== WATTMETER TEST (DFRobot SEN0291 = INA219 + 10 mOhm shunt), address 0x" << std::hex << addr << std::dec << " ===\n"
              << "The motor is NOT driven. Leave it idle (or running by hand) while this runs.\n\nStep 1: which I2C bus answers?\n";
    int bus = -1;
    for (int b : {Config::I2C_BUS_HARDWARE, Config::I2C_BUS_EXTRA}) {
        const bool ok = i2c_device_answers(pi, b, addr);
        std::cout << "   /dev/i2c-" << b << "  0x" << std::hex << addr << std::dec << " : " << (ok ? "ANSWERS" : "no answer") << '\n';
        if (ok && bus < 0) bus = b;
    }
    if (forced_bus >= 0) { bus = forced_bus; std::cout << "   (forced to /dev/i2c-" << bus << ")\n"; }
    if (bus < 0 || !i2c_device_answers(pi, bus, addr)) {
        std::cout << "\nRESULT: FAIL - no device at 0x" << std::hex << addr << std::dec << ".\n"
                  << "  * check VCC (3.3 or 5 V), GND, SDA, SCL on the module, and that the SDA/SCL wires go to the right GPIOs (bus 3: GPIO 8 = SDA, GPIO 7 = SCL)\n"
                  << "  * the extra bus needs pull-ups (4.7 kOhm to 3.3 V on SDA and SCL) unless the module already has them\n"
                  << "  * check the DIP switch address (0x40 / 0x41 / 0x44 / 0x45): run  i2cdetect -y 1  and  i2cdetect -y 3  and use  --power-addr=0x..\n"
                  << "  * check that  dtoverlay=i2c-gpio,bus=3,...  is active:  ls /dev/i2c-*\n";
        pigpio_stop(pi);
        return 1;
    }
    PowerMeter meter(pi, bus, addr, shunt, i_gain, i_off, v_gain);
    std::cout << "\nStep 2: does it behave like an INA219 (configuration register write + read-back)? ";
    if (!meter.available()) {
        std::cout << "NO\n\nRESULT: FAIL - something answers at 0x" << std::hex << addr << std::dec << " on /dev/i2c-" << bus
                  << " but it is not accepting the INA219 configuration (wrong chip at this address, or a noisy / marginal bus).\n";
        pigpio_stop(pi);
        return 1;
    }
    std::cout << "yes (found on /dev/i2c-" << bus << ")\n\nStep 3: reading for " << seconds << " s (10 reads per second; one line every 0.5 s)\n"
              << "   t(s)    bus V     current mA    power W    read ms\n";
    std::vector<double> tv, vv, iv, rv;
    long fails = 0;
    std::ofstream csv(csv_path);
    csv << "time_s,bus_v,current_ma,power_w,read_ms,ok\n";
    const auto t0 = std::chrono::steady_clock::now();
    auto next = t0;
    int printed = 0;
    while (run_loop) {
        next += std::chrono::milliseconds(100);
        std::this_thread::sleep_until(next);
        const auto a = std::chrono::steady_clock::now();
        double v = 0.0, ma = 0.0;
        const bool ok = meter.read(v, ma);
        const double rms = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - a).count();
        const double t = std::chrono::duration<double>(a - t0).count();
        if (t > seconds) break;
        if (ok) { tv.push_back(t); vv.push_back(v); iv.push_back(ma); rv.push_back(rms); } else { ++fails; }
        csv << std::fixed << std::setprecision(3) << t << ',';
        if (ok) csv << v << ',' << ma << ',' << (v * ma / 1000.0); else csv << ",,";
        csv << ',' << rms << ',' << (ok ? 1 : 0) << '\n';
        if (t >= 0.5 * printed) {
            ++printed;
            if (ok) std::cout << std::fixed << std::setprecision(2) << "  " << std::setw(5) << t << "  " << std::setw(8) << v << "  " << std::setw(11) << std::setprecision(0) << ma
                              << "  " << std::setw(9) << std::setprecision(2) << (v * ma / 1000.0) << "  " << std::setw(8) << std::setprecision(2) << rms << '\n';
            else std::cout << "  " << std::fixed << std::setprecision(2) << std::setw(5) << t << "   READ FAILED\n";
        }
    }
    csv.close();
    pigpio_stop(pi);
    if (vv.size() < 5) { std::cout << "\nRESULT: FAIL - fewer than 5 successful reads (" << fails << " failed).\n"; return 1; }
    auto stats = [](const std::vector<double>& x, double& mean, double& sd, double& mn, double& mx) {
        mean = 0.0; for (double e : x) mean += e; mean /= static_cast<double>(x.size());
        sd = 0.0; for (double e : x) sd += (e - mean) * (e - mean); sd = std::sqrt(sd / static_cast<double>(x.size()));
        mn = *std::min_element(x.begin(), x.end()); mx = *std::max_element(x.begin(), x.end());
    };
    double vm, vs, vmin, vmax, im, is, imin, imax, rm, rsd, rmin, rmax;
    stats(vv, vm, vs, vmin, vmax); stats(iv, im, is, imin, imax); stats(rv, rm, rsd, rmin, rmax);
    std::cout << std::fixed << "\nSummary over " << vv.size() << " reads (" << fails << " failed):\n"
              << std::setprecision(3) << "   bus voltage : mean " << vm << " V   std " << vs << "   min " << vmin << "   max " << vmax << '\n'
              << std::setprecision(1) << "   current     : mean " << im << " mA  std " << is << "   min " << imin << "   max " << imax << '\n'
              << std::setprecision(2) << "   power       : mean " << (vm * im / 1000.0) << " W\n"
              << "   read time   : mean " << rm << " ms   max " << rmax << " ms\n\nChecks:\n";
    bool pass = true;
    auto check = [&](bool ok, const std::string& good, const std::string& bad) { std::cout << "   [" << (ok ? " OK " : "FAIL") << "] " << (ok ? good : bad) << '\n'; if (!ok) pass = false; };
    check(fails == 0, "no failed reads", std::to_string(fails) + " reads failed: the bus is marginal (pull-ups? wire length? loose connector?)");
    check(vm >= 0.5, "bus voltage present", "bus voltage is about 0 V: the module is not in series with a powered supply (supply + -> IN+, IN- -> the motor driver's +)");
    check(vm <= 26.0, "bus voltage within the 26 V rating", "bus voltage above the module's 26 V rating");
    check(im > -20.0, "current is not negative", "current is negative: IN+ and IN- are swapped");
    check(std::abs(imax) < 8000.0 && std::abs(imin) < 8000.0, "current within the 8 A range", "current beyond the 8 A range");
    check(vs < 0.05 && is < 20.0, "readings are stable (voltage std < 0.05 V, current std < 20 mA while idle)", "readings are noisy for a steady load: check grounding and wiring, or the motor was changing speed");
    check(rm < 5.0 && rmax < 50.0, "reads are fast (do not disturb the control loop)", "reads are slow: the bus is slow or busy");
    std::cout << "\n   idle current of the driver board: " << std::setprecision(1) << im << " mA at " << std::setprecision(2) << vm << " V.  Now run one step test to see the current rise with duty:\n"
              << "       sudo env POWER=1 ./run_steps.sh map --label wmcheck --duties \"10 20\" --map-duration 10 --pause 10\n"
              << "   (its table has 'power W' and 'current mA' columns; they must INCREASE with the duty)\n"
              << "\nData written to " << csv_path << "\nRESULT: " << (pass ? "PASS" : "CHECK - see the FAIL lines above") << '\n';
    return pass ? 0 : 1;
}

int main(int argc, char** argv) {
    acquire_single_instance_lock();
    bool debug_motor = false;                       // --debug-motor: print the PWM the daemon commands (about once per second)
    double dbg_loop_max_ms = 0.0, dbg_lcd_max_ms = 0.0;   // --debug-motor timing probes (loop period, OLED redraw time)
    bool no_lcd = false;                            // --no-lcd: do not touch the I2C display at all (isolates a hung I2C bus / display)
    int lcd_bus_forced = -1;                        // --lcd-bus=N: use /dev/i2c-N for the display (default: auto-detect)
    bool use_power = true;                          // --no-power: do not touch the wattmeter in normal mode
    int power_addr = Config::I2C_POWER_ADDR;        // --power-addr=0x45
    int power_bus_forced = -1;                      // --power-bus=N: use /dev/i2c-N for the wattmeter (default: auto-detect)
    double power_shunt_ohm = 0.010, power_i_gain = 1.0, power_i_offset_ma = 0.0, power_v_gain = 1.0;
    // RS485 torque amplifier in normal mode: OFF unless --torque is passed (same flags as test mode). Default port = the USB RS485 adapter (/dev/ttyUSB0).
    bool use_torque = false;
    TorqueSensor::Settings torque_cfg;
    torque_cfg.port = "/dev/ttyUSB0"; torque_cfg.baud = 9600; torque_cfg.slave_id = 1; torque_cfg.reg_addr = 0; torque_cfg.reg_count = 2;
    torque_cfg.value_format = "int32"; torque_cfg.word_order = "big"; torque_cfg.scale = 0.001; torque_cfg.offset = 0.0; torque_cfg.stop_bits = 1;
    bool do_tare = true;                            // --no-tare: do not zero the torque amplifier at start-up
    int tare_settle_ms = 1500;                      // --tare-settle-ms=N: wait after the zero command (amplifier digital filter)
    int torque_period_ms = 50;                      // --torque-period-ms=N: background poll period (50 -> 20 Hz)
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--torque") use_torque = true;
        if (a == "--no-torque") use_torque = false;
        if (a.rfind("--torque-port=", 0) == 0) torque_cfg.port = a.substr(14);
        if (a.rfind("--torque-baud=", 0) == 0) torque_cfg.baud = std::stoi(a.substr(14));
        if (a.rfind("--torque-slave=", 0) == 0) torque_cfg.slave_id = std::stoi(a.substr(15));
        if (a.rfind("--torque-reg=", 0) == 0) torque_cfg.reg_addr = static_cast<unsigned>(std::stoul(a.substr(13)));
        if (a.rfind("--torque-regcount=", 0) == 0) torque_cfg.reg_count = static_cast<unsigned>(std::stoul(a.substr(18)));
        if (a.rfind("--torque-format=", 0) == 0) torque_cfg.value_format = a.substr(16);
        if (a.rfind("--torque-word-order=", 0) == 0) torque_cfg.word_order = a.substr(20);
        if (a.rfind("--torque-scale=", 0) == 0) torque_cfg.scale = std::stod(a.substr(15));
        if (a.rfind("--torque-offset=", 0) == 0) torque_cfg.offset = std::stod(a.substr(16));
        if (a.rfind("--torque-stopbits=", 0) == 0) torque_cfg.stop_bits = std::stoi(a.substr(18));
        if (a.rfind("--torque-period-ms=", 0) == 0) torque_period_ms = std::max(10, std::stoi(a.substr(19)));
        if (a == "--no-tare") do_tare = false;
        if (a == "--tare") do_tare = true;
        if (a.rfind("--tare-settle-ms=", 0) == 0) tare_settle_ms = std::max(200, std::stoi(a.substr(17)));
        if (a == "--no-power") use_power = false;
        if (a == "--power") use_power = true;
        if (a.rfind("--power-addr=", 0) == 0) power_addr = std::stoi(a.substr(13), nullptr, 0);
        if (a.rfind("--power-bus=", 0) == 0) power_bus_forced = std::stoi(a.substr(12));
        if (a.rfind("--power-shunt-ohm=", 0) == 0) power_shunt_ohm = std::stod(a.substr(18));
        if (a.rfind("--power-i-gain=", 0) == 0) power_i_gain = std::stod(a.substr(15));
        if (a.rfind("--power-i-offset-ma=", 0) == 0) power_i_offset_ma = std::stod(a.substr(20));
        if (a.rfind("--power-v-gain=", 0) == 0) power_v_gain = std::stod(a.substr(15));
        if (a == "--debug-motor") debug_motor = true;
        if (a == "--no-lcd") no_lcd = true;
        if (a.rfind("--lcd-bus=", 0) == 0) lcd_bus_forced = std::stoi(a.substr(10));
    }
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--test-wattmeter") return run_wattmeter_test(argc, argv);
        if (std::string(argv[i]) == "--test-torque") return run_torque_test(argc, argv);
    }
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--cpr=", 0) == 0) {
            Config::ENCODER_CPR = std::stod(arg.substr(6));
            std::cout << "[CONFIG] Set ENCODER_CPR to " << Config::ENCODER_CPR << '\n';
        } else if (arg.rfind("--window=", 0) == 0) {
            Config::RPM_SAMPLE_WINDOW_US = std::stoi(arg.substr(9));
            std::cout << "[CONFIG] Set RPM_SAMPLE_WINDOW_US to " << Config::RPM_SAMPLE_WINDOW_US << "us\n";
        } else if (arg.rfind("--kp=", 0) == 0) {
            Config::GLOBAL_KP = std::stod(arg.substr(5));
            std::cout << "[CONFIG] Set Kp to " << Config::GLOBAL_KP << '\n';
        } else if (arg.rfind("--ki=", 0) == 0) {
            Config::GLOBAL_KI = std::stod(arg.substr(5));
            std::cout << "[CONFIG] Set Ki to " << Config::GLOBAL_KI << '\n';
        } else if (arg.rfind("--alpha=", 0) == 0) {
            Config::FEEDBACK_ALPHA = std::stod(arg.substr(8));
            std::cout << "[CONFIG] Set FEEDBACK_ALPHA to " << Config::FEEDBACK_ALPHA << '\n';
        }
    }

    if (argc > 1 && std::string(argv[1]) == "--test") {
        TestOptions options;
        if (!parse_test_options(argc, argv, options)) {
            std::cerr << "Usage: ./mixr1_daemon --test ...\n";
            return 2;
        }
        if (options.ref_step_to2 > 0.0) std::cout << "[CONFIG] 2nd step: -> " << options.ref_step_to2 << " RPM at t=" << options.ref_step_at2 << " s\n";
        if (options.power_warn_w > 0.0) std::cout << "[CONFIG] Power warning threshold: " << options.power_warn_w << " W\n";
        if (options.torque) std::cout << "[CONFIG] Torque sensor: " << options.torque_port << " slave " << options.torque_slave_id
                                       << ", reg " << options.torque_reg_addr << " x" << options.torque_reg_count
                                       << " (" << options.torque_format << ")\n";
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        std::signal(SIGHUP, signal_handler);   // SSH disconnect: stop the motor cleanly instead of dying with PWM still running
        return run_test(options);
    }

    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) {
        std::cerr << "[CRITICAL] Failed to connect to pigpiod.\n";
        return 1;
    }

    cpu_set_t cpuset;
    CPU_ZERO(&cpuset);
    CPU_SET(3, &cpuset);
    if (pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset) != 0) {
        std::cerr << "[WARNING] Failed to set CPU affinity to Core 3.\n";
    }

    sched_param sch;
    int policy;
    pthread_getschedparam(pthread_self(), &policy, &sch);
    sch.sched_priority = 90;
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sch) != 0) {
        std::cerr << "[WARNING] Failed to set SCHED_FIFO. Must run with sudo.\n";
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGHUP, signal_handler);   // SSH disconnect: stop the motor cleanly
    std::signal(SIGPIPE, SIG_IGN); 

    auto network = std::make_unique<TelemetryServer>();
    if (!network->start_server(Config::TCP_PORT)) {
        std::cerr << "CRITICAL: port " << Config::TCP_PORT << " is already in use: another mixr1_daemon is running (or was SUSPENDED with Ctrl+Z; a suspended daemon keeps the port and the motor PWM).\n"
                  << "  See it:  pgrep -a mixr1_daemon        Stop it:  sudo ./motor_off.sh\n";
        pigpio_stop(pi);
        return 1;
    }

    std::cout << "[MIXR-1] Waiting for Dashboard (Port " << Config::TCP_PORT << ")...\n";

    KinematicsEngine kinematics;
    AMT102Encoder encoder(pi, Config::PIN_ENC_A, Config::PIN_ENC_B, Config::PIN_ENC_X);
    MotorController motor(pi);
    std::cout << "[MIXR-1] Motor driver initialised: commanded PWM = 0 %.  (start with --debug-motor to print every PWM the daemon commands)\n";
    const int display_addr = (Config::DISPLAY_TYPE == Config::DisplayType::SSD1306_128x64) ? Config::I2C_OLED_ADDR : Config::I2C_LCD_ADDR;
    int display_bus = Config::I2C_BUS_DISPLAY;
    const bool spi_display = (Config::DISPLAY_TYPE == Config::DisplayType::ST7789_320x240);
    if (no_lcd) {
        std::cout << "[MIXR-1] --no-lcd: the display is NOT used.\n";
    } else if (spi_display) {
        std::cout << "[MIXR-1] ST7789 SPI display: DC=GPIO " << Config::PIN_TFT_DC << ", RES=GPIO " << Config::PIN_TFT_RES << ", CS=GPIO " << Config::PIN_TFT_CS << " (no I2C probe).\n";
    } else {
        display_bus = (lcd_bus_forced >= 0) ? lcd_bus_forced
                                            : find_i2c_bus(pi, display_addr, Config::I2C_BUS_DISPLAY, {Config::I2C_BUS_HARDWARE, Config::I2C_BUS_EXTRA});
        if (display_bus >= 0 && i2c_device_answers(pi, display_bus, display_addr)) {
            std::cout << "[MIXR-1] Display found at 0x" << std::hex << display_addr << std::dec << " on /dev/i2c-" << display_bus << ".\n";
        } else {
            std::cout << "[MIXR-1] No display answered at 0x" << std::hex << display_addr << std::dec << " on /dev/i2c-" << Config::I2C_BUS_HARDWARE << " or /dev/i2c-" << Config::I2C_BUS_EXTRA
                      << ": running WITHOUT a display (a missing display would otherwise stall the loop with failing I2C writes).\n";
            no_lcd = true;
            display_bus = Config::I2C_BUS_DISPLAY;
        }
    }
    LCD1602 lcd(pi, Config::DISPLAY_TYPE, no_lcd ? 0xFFFF : display_addr, display_bus);       // an invalid address makes i2c_open fail at once: every display call becomes a no-op
    EC11Input ec11(pi, Config::PIN_EC11_A, Config::PIN_EC11_B, Config::PIN_EC11_SW);
    PushButton start_key(pi, Config::PIN_START_KEY);               // module KO key: toggles RUN / STOP in standalone mode
    PIController pi_control;

    // Wattmeter (INA219 on its own I2C bus): read at 20 Hz and forwarded to the dashboard. A missing meter is not an error: the dashboard just gets NaN.
    std::unique_ptr<PowerMeter> meter;
    if (use_power) {
        const int power_bus = (power_bus_forced >= 0)
            ? power_bus_forced
            : find_i2c_bus(pi, power_addr, Config::I2C_BUS_POWER, {Config::I2C_BUS_HARDWARE, Config::I2C_BUS_EXTRA});
        if (power_bus >= 0) meter = std::make_unique<PowerMeter>(pi, power_bus, power_addr, power_shunt_ohm, power_i_gain, power_i_offset_ma, power_v_gain);
        if (meter && meter->available()) {
            std::cout << "[MIXR-1] Wattmeter found at 0x" << std::hex << power_addr << std::dec << " on /dev/i2c-" << power_bus << ": streaming V / mA / W to the dashboard.\n";
        } else {
            std::cout << "[MIXR-1] No wattmeter answered at 0x" << std::hex << power_addr << std::dec << ": the dashboard will show no electrical power.\n";
            meter.reset();
        }
    }
    // Torque sensor (RS485 Modbus): a Modbus request + reply at 9600 baud takes ~20 ms, which would stall the 10 ms control loop, so it is polled
    // in a background thread. The thread is moved OFF the control core and back to normal scheduling (std::thread would inherit core 3 + SCHED_FIFO).
    std::unique_ptr<ModbusTorque> tsensor;
    std::atomic<double> torque_value{std::numeric_limits<double>::quiet_NaN()};
    std::atomic<bool> torque_stop{false};
    std::thread torque_thread;
    if (use_torque) {
        ModbusTorque::Settings mt;
        mt.port = torque_cfg.port; mt.baud = torque_cfg.baud; mt.slave_id = torque_cfg.slave_id;
        mt.reg_addr = torque_cfg.reg_addr; mt.reg_count = torque_cfg.reg_count; mt.value_format = torque_cfg.value_format;
        mt.word_order = torque_cfg.word_order; mt.scale = torque_cfg.scale; mt.offset = torque_cfg.offset; mt.stop_bits = torque_cfg.stop_bits;
        tsensor = std::make_unique<ModbusTorque>(mt);
        if (tsensor->available()) {
            std::cout << "[MIXR-1] Torque sensor answering on " << torque_cfg.port << " (slave " << torque_cfg.slave_id << ", reg " << torque_cfg.reg_addr
                      << " x" << torque_cfg.reg_count << ", " << torque_cfg.value_format << "): streaming torque to the dashboard.\n";
            // Zero the amplifier at no load BEFORE the polling thread starts (nothing else may use the port during the zero command).
            if (do_tare) zero_torque_at_rest(*tsensor, encoder, tare_settle_ms, 2.0 * torque_cfg.scale, "[MIXR-1]");
            torque_thread = std::thread([&]() {
                cpu_set_t others;
                CPU_ZERO(&others);
                for (int c = 0; c < 3; ++c) CPU_SET(c, &others);
                pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &others);
                sched_param normal{};
                normal.sched_priority = 0;
                pthread_setschedparam(pthread_self(), SCHED_OTHER, &normal);
                int fails = 0;
                auto next = std::chrono::steady_clock::now();
                while (!torque_stop.load()) {
                    next += std::chrono::milliseconds(torque_period_ms);
                    std::this_thread::sleep_until(next);
                    double tq = 0.0;
                    if (tsensor->read(tq)) { torque_value.store(tq); fails = 0; }
                    else if (++fails > 10) torque_value.store(std::numeric_limits<double>::quiet_NaN());   // no stale value after an RS485 dropout
                }
            });
        } else {
            std::cout << "[MIXR-1] No torque sensor answered on " << torque_cfg.port << " (slave " << torque_cfg.slave_id
                      << "): " << tsensor->last_error() << "\n    check port, baud, slave ID, register and wiring (python3 tools/rs485_probe.py finds them). Running without torque.\n";
            tsensor.reset();
        }
    }
    PowerSample power_sample;                       // NaN until the first good reading
    int power_loop_counter = 0;
    int power_fail_streak = 0;

    bool dashboard_connected = false;
    bool simulink_is_active = false;
    bool mode3_notified = false;
    bool prev_open_loop = false;
    double standalone_target_rpm = 0.0;
    bool standalone_running = Config::STANDALONE_RUN_AT_BOOT;      // KO key toggles this; the knob target only drives the motor while true
    double dashboard_target_rpm = 0.0;
    int dashboard_target_pwm_pct = 0;
    bool dashboard_pi_mode = false;
    int current_pwm = 0;
    int simulink_check_counter = Config::SIMULINK_CHECK_INTERVAL;
    int lcd_prescaler = 0;
    int ec11_accum = 0;
    int ec11_last_dir = 0;
    auto ec11_last_edge = std::chrono::steady_clock::now();
    double ec11_last_step = 0.0;
    double ec11_rate_cps = 0.0;
    auto ec11_last_click = std::chrono::steady_clock::now() - std::chrono::seconds(10);
    double last_rpm = 0.0;
    std::string last_lines[4];

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    kinematics.reset(encoder.get_sync_snapshot());
    auto next_wake = std::chrono::steady_clock::now();
    auto last_time = next_wake;
    const auto loop_start = next_wake;

    while (run_loop) {
        next_wake += std::chrono::microseconds(Config::LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);

        const auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = current_time - last_time;
        last_time = current_time;
        if (debug_motor) dbg_loop_max_ms = std::max(dbg_loop_max_ms, dt.count() * 1000.0);

        network->poll_for_client();

        if (++simulink_check_counter >= Config::SIMULINK_CHECK_INTERVAL) {
            simulink_check_counter = 0;
            simulink_is_active = ProcessMonitor::is_simulink_running();
        }

        if (simulink_is_active) {
            if (!mode3_notified) {
                std::cout << "[MIXR-1] MATLAB detected. Releasing motor control...\n";
                mode3_notified = true;
                pi_control.reset();
            }
            motor.set_pwm(0);
            if (network->has_client()) network->send_packet(-2.0, -2.0, -2);
            continue;
        }

        if (mode3_notified) {
            std::cout << "[MIXR-1] MATLAB teardown complete.\n";
            mode3_notified = false;
            kinematics.reset(encoder.get_sync_snapshot());
            pi_control.reset();
        }

        // ---- Connection edge: the dashboard owns control while connected ----
        const bool client_now = network->has_client();
        if (client_now != dashboard_connected) {
            dashboard_connected = client_now;
            pi_control.reset();
            if (client_now) {
                dashboard_target_rpm = 0.0;
                dashboard_target_pwm_pct = 0;
                dashboard_pi_mode = false;
                standalone_target_rpm = 0.0;   // knob preview restarts from 0
                ec11_accum = 0;
                std::cout << "[MIXR-1] MODE2: Dashboard connected (EC11 LOCKED, log only)\n";
            } else {
                standalone_target_rpm = 0.0;   // do not inherit the dashboard's speed
                standalone_running = Config::STANDALONE_RUN_AT_BOOT;
                std::cout << "[MIXR-1] MODE1: Standalone (EC11 active)\n";
            }
        }
        if (client_now) {
            network->receive_command(dashboard_target_rpm, dashboard_target_pwm_pct, dashboard_pi_mode);
        }
        const bool dashboard_active = network->has_client();

        // ---- EC11: drives the motor only in standalone mode. When the dashboard is
        //      connected the knob is locked, but every event is still printed to the terminal. ----
        auto log_states = [&](const char* event) {
            const double est_torque_nm = (static_cast<double>(current_pwm) / 4095.0) * Config::TORQUE_ESTIMATE_MAX_NM;
            std::cout << std::fixed << std::setprecision(0)
                      << "[EC11] " << event
                      << " | " << (dashboard_active ? "MODE2 DASHBOARD (knob locked)" : "MODE1 STANDALONE")
                      << " | knob_target=" << standalone_target_rpm << " RPM"
                      << " (step " << ec11_last_step << ")";
            if (dashboard_active) {
                if (dashboard_pi_mode) std::cout << " | dash_target=" << dashboard_target_rpm << " RPM";
                else                   std::cout << " | dash_target=" << dashboard_target_pwm_pct << " %PWM";
            }
            std::cout << " | measured=" << last_rpm << " RPM"
                      << " | pwm=" << (current_pwm * 100 / 4095) << "%"
                      << " | torque~" << std::setprecision(2) << est_torque_nm << " Nm\n";
        };

        if (start_key.pressed() && !dashboard_active) {    // KO key = start / stop the motor (standalone mode only)
            standalone_running = !standalone_running;
            pi_control.reset();
            log_states(standalone_running ? "START   " : "STOP    ");
        }

        if (ec11.button_pressed()) {                       // button = zero the knob target
            standalone_target_rpm = 0.0;
            if (!dashboard_active) pi_control.reset();
            log_states("BUTTON  ");
        }

        {
            const int d = ec11.read_delta();               // interrupt-accumulated, nothing is lost
            if (d != 0) {
                ec11_accum += d;
                ec11_last_edge = std::chrono::steady_clock::now();
            } else if (ec11_accum != 0 &&
                       std::chrono::steady_clock::now() - ec11_last_edge > std::chrono::milliseconds(300)) {
                ec11_accum = 0;                            // drop a stale half-click so it can't drift
            }
        }
        if (std::abs(ec11_accum) >= Config::EC11_TRANSITIONS_PER_CLICK) {
            const int clicks = ec11_accum / Config::EC11_TRANSITIONS_PER_CLICK;
            ec11_accum -= clicks * Config::EC11_TRANSITIONS_PER_CLICK;

            // Smooth velocity response: measure click rate, map it continuously to a step size.
            const int dir = (clicks > 0) ? 1 : -1;
            const auto now_click = std::chrono::steady_clock::now();
            const double gap_ms = std::max(
                1.0, static_cast<double>(std::chrono::duration_cast<std::chrono::microseconds>(now_click - ec11_last_click).count()) / 1000.0);
            ec11_last_click = now_click;

            if ((dir != ec11_last_dir && ec11_last_dir != 0) || gap_ms > Config::EC11_IDLE_RESET_MS) {
                ec11_rate_cps = 0.0;                              // reversal or pause -> start fine again
            }
            const double inst_rate = (std::abs(clicks) * 1000.0) / gap_ms;   // clicks per second
            const double a = Config::EC11_RATE_SMOOTH;
            ec11_rate_cps = (ec11_rate_cps == 0.0) ? std::min(inst_rate, 2.0) : (a * inst_rate + (1.0 - a) * ec11_rate_cps);

            const double t = std::min(1.0, std::pow(ec11_rate_cps / Config::EC11_RATE_FULL_CPS, Config::EC11_CURVE_GAMMA));
            const double step_raw = Config::EC11_STEP_MIN_RPM + (Config::EC11_STEP_MAX_RPM - Config::EC11_STEP_MIN_RPM) * t;
            const int step_rpm = std::max(1, static_cast<int>(std::lround(step_raw)));   // whole RPM only

            ec11_last_dir = dir;
            ec11_last_step = step_rpm;

            standalone_target_rpm = std::clamp(
                standalone_target_rpm + static_cast<double>(clicks * step_rpm),
                Config::STANDALONE_MIN_RPM, Config::STANDALONE_MAX_RPM);
            log_states(dir > 0 ? "TURN CW " : "TURN CCW");
        }

        const bool update_lcd = (++lcd_prescaler >= Config::LCD_PRESCALER);
        if (update_lcd) lcd_prescaler = 0;

        auto state = kinematics.process(encoder.get_sync_snapshot(), current_pwm, update_lcd);
        last_rpm = state.exact_rpm;

        const bool open_loop = dashboard_active && !dashboard_pi_mode;
        if (open_loop != prev_open_loop) {
            pi_control.reset();
            prev_open_loop = open_loop;
        }

        double active_target = 0.0;      // PI target actually being tracked (0 in open loop)
        if (open_loop) {
            current_pwm = std::clamp((dashboard_target_pwm_pct * 4095) / 100, 0, 4095);
            motor.set_pwm(current_pwm);
        } else {
            const double target = dashboard_active
                ? std::clamp(dashboard_target_rpm, Config::STANDALONE_MIN_RPM, Config::STANDALONE_MAX_RPM)
                : (standalone_running ? standalone_target_rpm : 0.0);
            active_target = target;
            if (target > 0.0) {
                current_pwm = pi_control.compute(target, state.exact_rpm, dt.count());
            } else {
                current_pwm = 0;
                pi_control.reset();      // drop stale filter + integral so the next start is clean
            }
            motor.set_pwm(current_pwm);
        }

        if (debug_motor) {
            static int dbg_counter = 0;
            static int dbg_last_pwm = -1;
            const bool dbg_periodic = ((dbg_counter++ % 100) == 0);
            if (dbg_periodic) {
                std::cout << "[TIMING] loop period max " << std::fixed << std::setprecision(1) << dbg_loop_max_ms << " ms (ideal 10) | OLED redraw max "
                          << dbg_lcd_max_ms << " ms  <- if this is large, the I2C display/bus is stalling the loop (and the dashboard)\n";
                dbg_loop_max_ms = 0.0;
                dbg_lcd_max_ms = 0.0;
            }
            if (dbg_periodic || std::abs(current_pwm - dbg_last_pwm) > 200) {
                dbg_last_pwm = current_pwm;
                std::cout << "[DEBUG] " << (dashboard_active ? "DASHBOARD" : "STANDALONE") << " | knob_target=" << standalone_target_rpm
                          << " RPM, dashboard_target=" << (dashboard_pi_mode ? dashboard_target_rpm : static_cast<double>(dashboard_target_pwm_pct))
                          << (dashboard_pi_mode ? " RPM" : " %PWM") << " | COMMANDED PWM = " << (current_pwm * 100 / 4095) << " % (" << current_pwm
                          << " counts) | measured = " << state.exact_rpm << " RPM\n";
            }
        }

        if (meter && ++power_loop_counter >= Config::POWER_READ_DIV) {
            power_loop_counter = 0;
            double v = 0.0, ma = 0.0;
            if (meter->read(v, ma)) {
                power_sample.bus_v = v; power_sample.current_ma = ma; power_sample.power_w = v * ma / 1000.0;
                power_fail_streak = 0;
            } else if (++power_fail_streak > 10) {          // do not keep sending a stale value after an I2C dropout
                power_sample = PowerSample{};
            }
        }

        if (dashboard_active) {
            bool sent;
            if (network->wants_ext()) {
                // Extended packet: raw, display EMA (same smoothed trace as the 3-field packet), revs, PWM %, PI target, uptime
                const double uptime = std::chrono::duration<double>(current_time - loop_start).count();
                sent = network->send_packet_ext(state.exact_rpm, state.ema_filtered_rpm,
                                                encoder.get_revolutions(), current_pwm * 100.0 / 4095.0,
                                                active_target, uptime, power_sample, torque_value.load());
            } else {
                sent = network->send_packet(state.exact_rpm, state.ema_filtered_rpm, encoder.get_revolutions());
            }
            (void)sent;      // a dead / stalled dashboard is dropped (and reported once) inside TelemetryServer; no per-loop message
        }

        if (update_lcd) {
            const auto lcd_t0 = std::chrono::steady_clock::now();
            const double est_torque_nm = (static_cast<double>(current_pwm) / 4095.0) * Config::TORQUE_ESTIMATE_MAX_NM;
            std::ostringstream l1, l2, l3, l4;
            l1 << (dashboard_active ? "MODE2 DASHBOARD" : (standalone_running ? "MODE1 RUN" : "MODE1 STOPPED"));
            if (open_loop) {
                l2 << "SET: " << dashboard_target_pwm_pct << " %";
            } else {
                l2 << std::fixed << std::setprecision(0) << "SET: "
                   << (dashboard_active ? dashboard_target_rpm : standalone_target_rpm) << " RPM";
            }
            l3 << std::fixed << std::setprecision(0) << "READ: " << state.exact_rpm << " RPM";
            l4 << std::fixed << std::setprecision(2) << "T:" << est_torque_nm
               << "NM PWM:" << (current_pwm * 100 / 4095) << "%";

            const std::string lines[4] = {l1.str(), l2.str(), l3.str(), l4.str()};
            const int step = (lcd.row_count() >= 8) ? 2 : 1;   // OLED: rows 0,2,4,6
            const int n = (lcd.row_count() >= 4) ? 4 : 2;
            for (int i = 0; i < n; ++i) {
                if (lines[i] != last_lines[i]) {               // only redraw changed lines
                    lcd.print_line(i * step, lines[i]);
                    last_lines[i] = lines[i];
                }
            }
            dbg_lcd_max_ms = std::max(dbg_lcd_max_ms, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - lcd_t0).count());
        }
    }

    motor.stop_motor();
    torque_stop = true;
    if (torque_thread.joinable()) torque_thread.join();
    network->stop_server();

    pigpio_stop(pi);
    std::cout << "\n[MIXR-1] Daemon safely offline.\n";
    return 0;
}
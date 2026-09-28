// src/main.cpp
#include <iostream>
#include <csignal>
#include <atomic>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>
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

#include "config.hpp"
#include "process_monitor.hpp"
#include "kinematics.hpp"
#include "pi_controller.hpp"
#include "encoder.hpp"
#include "motor.hpp"
#include "lcd.hpp"
#include "network.hpp"
#include "ec11.hpp"

std::atomic<bool> run_loop{true};

void signal_handler(int signum) {
    run_loop = false;
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
        } else {
            return false;
        }
    }
    return options.duration_sec > 0.0 && options.target_rpm >= 0.0 &&
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

    log << "elapsed_s,step_index,pwm_percent,loop_period_us,late_us,raw_rpm,filtered_rpm,target_rpm,pwm,error_rpm,intended_mode,intended_fifo,fifo_active,condition\n";
    
    int current_pwm = options.use_pi ? 0 : (options.sweep ? 0 : options.fixed_pwm);
    double current_target = options.use_pi ? 0.0 : options.target_rpm;
    motor.set_pwm(current_pwm);

    std::this_thread::sleep_for(std::chrono::milliseconds(250));

    kinematics.reset(encoder.get_sync_snapshot());
    controller.reset();

    const auto start = std::chrono::steady_clock::now();
    auto next_wake = start;
    auto previous_tick = start;
    int step_index = -1;
    std::vector<double> periods_us;
    std::vector<double> late_us;
    std::vector<double> rpm_samples;
    std::vector<double> errors;

    const double total_duration = options.sweep ? options.duration_sec * 11.0 : options.duration_sec;
    
    if (options.sine_mode) {
        std::cout << "[TEST] Sine Wave Mode | Target: " << options.target_rpm << " RPM +/- " 
                  << options.sine_amplitude << " | Freq: " << options.sine_freq_hz << " Hz\n";
    }

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
        
        auto state = kinematics.process(encoder.get_sync_snapshot(), current_pwm, false);

        if (options.use_pi) {
            current_pwm = controller.compute(current_target, state.exact_rpm, period / 1000000.0);
            motor.set_pwm(current_pwm);
        }

        const double error = current_target - state.exact_rpm;
        const int pwm_percent = options.sweep ? step_index * 10 : (current_pwm * 100) / 4095;
        log << std::fixed << std::setprecision(6) << elapsed << ',' << step_index << ',' << pwm_percent << ','
            << period << ',' << lateness << ','
            << state.exact_rpm << ',' << state.ema_filtered_rpm << ',' << current_target << ','
            << current_pwm << ',' << error << ','
            << intended_mode << ',' << intended_fifo << ',' << (fifo_active ? "true" : "false") << ',' << condition << '\n';
        periods_us.push_back(period);
        late_us.push_back(lateness);
        rpm_samples.push_back(state.exact_rpm);
        errors.push_back(std::abs(error));
    }

    motor.stop_motor();
    pigpio_stop(pi);
    
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
    return 0;
}

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--cpr=", 0) == 0) {
            Config::ENCODER_CPR = std::stod(arg.substr(6));
            std::cout << "[CONFIG] Set ENCODER_CPR to " << Config::ENCODER_CPR << '\n';
        } else if (arg.rfind("--window=", 0) == 0) {
            Config::RPM_SAMPLE_WINDOW_US = std::stoi(arg.substr(9));
            std::cout << "[CONFIG] Set RPM_SAMPLE_WINDOW_US to " << Config::RPM_SAMPLE_WINDOW_US << "us\n";
        }
    }

    if (argc > 1 && std::string(argv[1]) == "--test") {
        TestOptions options;
        if (!parse_test_options(argc, argv, options)) {
            std::cerr << "Usage: ./mixr1_daemon --test ...\n";
            return 2;
        }
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
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
    std::signal(SIGPIPE, SIG_IGN); 

    auto network = std::make_unique<TelemetryServer>();
    if (!network->start_server(Config::TCP_PORT)) {
        std::cerr << "CRITICAL: Port locked.\n";
        pigpio_stop(pi);
        return 1;
    }

    std::cout << "[MIXR-1] Waiting for Dashboard (Port " << Config::TCP_PORT << ")...\n";

    KinematicsEngine kinematics;
    AMT102Encoder encoder(pi, Config::PIN_ENC_A, Config::PIN_ENC_B, Config::PIN_ENC_X);
    MotorController motor(pi);
    LCD1602 lcd(pi, Config::DISPLAY_TYPE);
    EC11Input ec11(pi, Config::PIN_EC11_A, Config::PIN_EC11_B, Config::PIN_EC11_SW);
    PIController pi_control;

    bool dashboard_connected = false;
    bool simulink_is_active = false;
    bool mode3_notified = false;
    bool prev_open_loop = false;
    double standalone_target_rpm = 0.0;
    double dashboard_target_rpm = 0.0;
    int dashboard_target_pwm_pct = 0;
    bool dashboard_pi_mode = false;
    int current_pwm = 0;
    int simulink_check_counter = Config::SIMULINK_CHECK_INTERVAL;
    int lcd_prescaler = 0;
    std::string last_lines[4];

    std::this_thread::sleep_for(std::chrono::milliseconds(250));
    kinematics.reset(encoder.get_sync_snapshot());
    auto next_wake = std::chrono::steady_clock::now();
    auto last_time = next_wake;

    while (run_loop) {
        next_wake += std::chrono::microseconds(Config::LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);

        const auto current_time = std::chrono::steady_clock::now();
        std::chrono::duration<double> dt = current_time - last_time;
        last_time = current_time;

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
                std::cout << "[MIXR-1] MODE2: Dashboard has control (EC11 disabled)\n";
            } else {
                standalone_target_rpm = 0.0;   // do not inherit the dashboard's speed
                std::cout << "[MIXR-1] MODE1: Standalone (EC11 active)\n";
            }
        }
        if (client_now) {
            network->receive_command(dashboard_target_rpm, dashboard_target_pwm_pct, dashboard_pi_mode);
        }
        const bool dashboard_active = network->has_client();

        if (!dashboard_active) {
            if (ec11.button_pressed()) {           // button = stop
                standalone_target_rpm = 0.0;
                pi_control.reset();
            }
            const int d = ec11.read_delta();
            if (d != 0) {
                standalone_target_rpm = std::clamp(
                    standalone_target_rpm + static_cast<double>(d * Config::EC11_RPM_STEP),
                    Config::STANDALONE_MIN_RPM, Config::STANDALONE_MAX_RPM);
            }
        } else {
            (void)ec11.read_delta();               // drain so nothing jumps on disconnect
            (void)ec11.button_pressed();
        }

        const bool update_lcd = (++lcd_prescaler >= Config::LCD_PRESCALER);
        if (update_lcd) lcd_prescaler = 0;

        auto state = kinematics.process(encoder.get_sync_snapshot(), current_pwm, update_lcd);

        const bool open_loop = dashboard_active && !dashboard_pi_mode;
        if (open_loop != prev_open_loop) {
            pi_control.reset();
            prev_open_loop = open_loop;
        }

        if (open_loop) {
            current_pwm = std::clamp((dashboard_target_pwm_pct * 4095) / 100, 0, 4095);
            motor.set_pwm(current_pwm);
        } else {
            const double target = dashboard_active
                ? std::clamp(dashboard_target_rpm, Config::STANDALONE_MIN_RPM, Config::STANDALONE_MAX_RPM)
                : standalone_target_rpm;
            if (target > 0.0) current_pwm = pi_control.compute(target, state.exact_rpm, dt.count());
            else              current_pwm = 0;
            motor.set_pwm(current_pwm);
        }

        if (dashboard_active) {
            if (!network->send_packet(state.exact_rpm, state.ema_filtered_rpm, encoder.get_revolutions())) {
                std::cout << "[MIXR-1] Dashboard send failed.\n";
            }
        }

        if (update_lcd) {
            const double est_torque_nm = (static_cast<double>(current_pwm) / 4095.0) * Config::TORQUE_ESTIMATE_MAX_NM;
            std::ostringstream l1, l2, l3, l4;
            l1 << (dashboard_active ? "MODE2 DASHBOARD" : "MODE1 STANDALONE");
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
        }
    }

    motor.stop_motor();
    network->stop_server();

    pigpio_stop(pi);
    std::cout << "\n[MIXR-1] Daemon safely offline.\n";
    return 0;
}
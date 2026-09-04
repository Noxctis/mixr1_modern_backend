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

std::atomic<bool> run_loop{true};

void signal_handler(int signum) {
    run_loop = false;
}

struct TestOptions {
    bool fifo = false;
    bool use_pi = false;
    bool sweep = true;
    double target_rpm = 1000.0;
    int fixed_pwm = 1000;
    double duration_sec = 10.0;
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
        } else if (argument == "--fixed") {
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
        } else {
            return false;
        }
    }
    return options.duration_sec > 0.0 && options.target_rpm >= 0.0 &&
           options.fixed_pwm >= 0 && options.fixed_pwm <= 4095;
}

int run_test(const TestOptions& options) {
    bool fifo_active = false;
    if (options.fifo) {
        fifo_active = set_fifo_priority();
        if (!fifo_active) std::cerr << "[TEST] SCHED_FIFO request failed; continuing without it.\n";
    }

    int pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) return 1;

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
    
    // UPDATED: Pass the snapshot to initialize the CET tracker
    kinematics.reset(encoder.get_snapshot());
    controller.reset();
    
    int current_pwm = options.use_pi ? 0 : options.fixed_pwm;
    double current_target = options.use_pi ? 0.0 : options.target_rpm;
    motor.set_pwm(current_pwm);

    const auto start = std::chrono::steady_clock::now();
    auto next_wake = start;
    auto previous_tick = start;
    int step_index = -1;
    std::vector<double> periods_us;
    std::vector<double> late_us;
    std::vector<double> rpm_samples;
    std::vector<double> errors;

    const double total_duration = options.sweep ? options.duration_sec * 11.0 : options.duration_sec;
    while (run_loop) {
        next_wake += std::chrono::microseconds(Config::LOOP_DELAY_US);
        std::this_thread::sleep_until(next_wake);
        const auto now = std::chrono::steady_clock::now();
        const double elapsed = std::chrono::duration<double>(now - start).count();
        if (elapsed >= total_duration) break;

        if (options.sweep) {
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
        
        // UPDATED: Process kinematics using the snapshot
        auto state = kinematics.process(encoder.get_snapshot(), current_pwm, false);

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
            std::cerr << "Usage: ./mixr1_daemon --test [--cpr=X] [--window=Y] [--sweep|--fixed] [--fifo|--no-fifo] [--pi|--no-pi] "
                         "[--target=RPM] [--pwm=0..4095] [--duration=SECONDS] [--csv=FILE]\n";
            return 2;
        }
        std::signal(SIGINT, signal_handler);
        std::signal(SIGTERM, signal_handler);
        return run_test(options);
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
        std::cerr << "[WARNING] Failed to set SCHED_FIFO. Must run with sudo for deterministic PI control.\n";
    }

    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    std::signal(SIGPIPE, SIG_IGN); 

    auto network = std::make_unique<TelemetryServer>();
    KinematicsEngine kinematics;

    while (run_loop) {
        if (!network->start_server(Config::TCP_PORT)) {
            std::cerr << "CRITICAL: Port locked. Retrying...\n";
            usleep(2000000);
            continue;
        }

        if (network->wait_for_client()) {
            std::cout << "[MIXR-1] Dashboard Connected.\n";
            
            int pi = -1;
            std::unique_ptr<AMT102Encoder> encoder = nullptr;
            std::unique_ptr<MotorController> motor = nullptr;
            std::unique_ptr<LCD1602> lcd = nullptr;
            
            PIController pi_control;
            
            bool mode3_notified = false;
            
            double target_rpm = 0.0;
            int target_pwm_pct = 0;
            bool pi_mode = true; 
            int current_pwm = 0;
            
            int simulink_check_counter = Config::SIMULINK_CHECK_INTERVAL; 
            int network_prescaler = 0;
            int lcd_prescaler = 0;
            bool simulink_is_active = false;

            auto next_wake = std::chrono::steady_clock::now();
            auto last_time = next_wake;

            while (run_loop) {
                next_wake += std::chrono::microseconds(Config::LOOP_DELAY_US);
                std::this_thread::sleep_until(next_wake);

                auto current_time = std::chrono::steady_clock::now();
                std::chrono::duration<double> dt = current_time - last_time;
                last_time = current_time;

                if (++simulink_check_counter >= Config::SIMULINK_CHECK_INTERVAL) {
                    simulink_check_counter = 0;
                    simulink_is_active = ProcessMonitor::is_simulink_running();
                }

                if (simulink_is_active) {
                    if (!mode3_notified) {
                        std::cout << "[MIXR-1] MATLAB detected. Releasing hardware...\n";
                        motor.reset();
                        encoder.reset();
                        lcd.reset();
                        pi_control.reset(); 
                        target_rpm = 0.0;
                        if (pi >= 0) { pigpio_stop(pi); pi = -1; } 
                        mode3_notified = true;
                    }
                    if (!network->send_packet(-2.0, -2.0, -2)) break; 
                    continue;
                }

                if (mode3_notified) {
                    std::cout << "[MIXR-1] MATLAB teardown complete.\n";
                    mode3_notified = false;
                }

                if (pi < 0) {
                    pi = pigpio_start(nullptr, nullptr);
                    if (pi >= 0) {
                        encoder = std::make_unique<AMT102Encoder>(pi, Config::PIN_ENC_A, Config::PIN_ENC_B, Config::PIN_ENC_X);
                        motor = std::make_unique<MotorController>(pi);
                        lcd = std::make_unique<LCD1602>(pi);
                        
                        // UPDATED: Pass snapshot
                        kinematics.reset(encoder->get_snapshot());
                        last_time = std::chrono::steady_clock::now();
                        next_wake = last_time;
                    } else {
                        continue;
                    }
                }

                if (network->receive_command(target_rpm, target_pwm_pct, pi_mode)) {
                    if (pi_mode && target_rpm <= 0.0) pi_control.reset();
                }

                bool update_net = (++network_prescaler >= Config::NETWORK_PRESCALER);
                if (update_net) network_prescaler = 0;

                bool update_lcd = (++lcd_prescaler >= Config::LCD_PRESCALER);
                if (update_lcd) lcd_prescaler = 0;

                // UPDATED: Process kinematics using snapshot
                auto state = kinematics.process(encoder->get_snapshot(), current_pwm, update_lcd);

                if (motor && !simulink_is_active) {
                    if (pi_mode) {
                        if (target_rpm > 0.0) {
                            current_pwm = pi_control.compute(target_rpm, state.exact_rpm, dt.count());
                            motor->set_pwm(current_pwm);
                        } else {
                            current_pwm = 0;
                            motor->set_pwm(0);
                        }
                    } else {
                        current_pwm = (target_pwm_pct * 4095) / 100;
                        motor->set_pwm(current_pwm);
                    }
                }

                if (update_net) {
                    if (!network->send_packet(state.exact_rpm, state.ema_filtered_rpm, encoder->get_revolutions())) break; 
                }

                if (update_lcd) {
                    if (lcd) {
                        std::ostringstream raw_str, filtered_str;
                        raw_str << std::fixed << std::setprecision(1) << "R:" << state.exact_rpm << " X:" << encoder->get_revolutions() << "   ";
                        filtered_str << std::fixed << std::setprecision(1) << "FLT: " << state.ema_filtered_rpm << "       ";
                        lcd->set_cursor(0, 0); lcd->print(raw_str.str());
                        lcd->set_cursor(1, 0); lcd->print(filtered_str.str());
                    }
                }
            }

            motor.reset();
            encoder.reset();
            lcd.reset();
            if (pi >= 0) { pigpio_stop(pi); pi = -1; }
        }
        network->stop_server();
    }

    std::cout << "\n[MIXR-1] Daemon safely offline.\n";
    return 0;
}
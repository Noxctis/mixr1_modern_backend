// src/main.cpp
#include <iostream>
#include <csignal>
#include <atomic>
#include <memory>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <pigpiod_if2.h>
#include <unistd.h>

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

int main() {
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
            
            // Controller now loads parameters safely from the configuration file
            PIController pi_control;
            
            bool mode3_notified = false;
            double target_rpm = 0.0;
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
                        
                        kinematics.reset(encoder->get_count());
                        last_time = std::chrono::steady_clock::now();
                        next_wake = last_time;
                    } else {
                        continue;
                    }
                }

                if (network->receive_command(target_rpm)) {
                    if (target_rpm <= 0.0) pi_control.reset();
                }

                bool update_net = (++network_prescaler >= Config::NETWORK_PRESCALER);
                if (update_net) network_prescaler = 0;

                bool update_lcd = (++lcd_prescaler >= Config::LCD_PRESCALER);
                if (update_lcd) lcd_prescaler = 0;

                auto state = kinematics.process(encoder->get_count(), current_pwm, update_lcd);

                if (motor && !simulink_is_active) {
                    if (target_rpm > 0.0) {
                        current_pwm = pi_control.compute(target_rpm, state.exact_rpm, dt.count());
                        motor->set_pwm(current_pwm);
                    } else {
                        current_pwm = 0;
                        motor->set_pwm(0);
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
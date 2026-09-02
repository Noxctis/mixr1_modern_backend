// src/network.cpp
#include "network.hpp"
#include "config.hpp"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <atomic>

extern std::atomic<bool> run_loop;

void TelemetryServer::disconnect_client() {
    if (client_socket >= 0) close(client_socket);
    client_socket = -1;
}

TelemetryServer::~TelemetryServer() { 
    stop_server(); 
}

bool TelemetryServer::start_server(int port) {
    server_fd = socket(AF_INET, SOCK_STREAM, 0);
    if (server_fd < 0) return false;
    
    int opt = 1;
    setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = INADDR_ANY;
    address.sin_port = htons(port);

    if (bind(server_fd, (struct sockaddr*)&address, sizeof(address)) < 0) return false;
    return listen(server_fd, 1) >= 0;
}

bool TelemetryServer::wait_for_client() {
    std::cout << "[MIXR-1] Waiting for Dashboard (Port " << Config::TCP_PORT << ")...\n";
    while (run_loop) {
        fd_set readfds;
        FD_ZERO(&readfds);
        FD_SET(server_fd, &readfds);
        struct timeval tv{1, 0}; 
        
        if (select(server_fd + 1, &readfds, nullptr, nullptr, &tv) > 0) {
            if (FD_ISSET(server_fd, &readfds)) {
                client_socket = accept(server_fd, nullptr, nullptr);
                
                int flag = 1;
                setsockopt(client_socket, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(int));
                rx_buffer.clear();
                return client_socket >= 0;
            }
        }
    }
    return false;
}

bool TelemetryServer::send_packet(double raw_rpm, double filtered_rpm, long long revolutions) const {
    if (client_socket < 0) return false;
    std::string packet = std::to_string(raw_rpm) + "," + std::to_string(filtered_rpm) + "," + std::to_string(revolutions) + "\n";
    return send(client_socket, packet.c_str(), packet.length(), MSG_NOSIGNAL) > 0;
}

bool TelemetryServer::receive_command(double& target_rpm, int& target_pwm_pct, bool& pi_mode) {
    if (client_socket < 0) return false;
    bool updated = false;
    char chunk[1024];

    while (true) {
        ssize_t bytes = recv(client_socket, chunk, sizeof(chunk) - 1, MSG_DONTWAIT);
        if (bytes > 0) {
            chunk[bytes] = '\0';
            rx_buffer += chunk;
        } else {
            break;
        }
    }

    size_t pos;
    while ((pos = rx_buffer.find('\n')) != std::string::npos) {
        std::string line = rx_buffer.substr(0, pos);
        rx_buffer.erase(0, pos + 1);

        size_t rpm_pos = line.find("CMD:RPM,");
        size_t pwm_pos = line.find("CMD:PWM,");
        size_t cpr_pos = line.find("CMD:CPR,");
        size_t win_pos = line.find("CMD:WIN,");

        if (rpm_pos != std::string::npos) {
            try {
                target_rpm = std::stod(line.substr(rpm_pos + 8));
                pi_mode = true;
                std::cout << "[MIXR-1] Closed-Loop PI Target: " << target_rpm << " RPM\n";
                updated = true;
            } catch (...) {}
        } else if (pwm_pos != std::string::npos) {
            try {
                target_pwm_pct = std::stoi(line.substr(pwm_pos + 8));
                pi_mode = false;
                int scaled_pwm = (target_pwm_pct * 4095) / 100;
                std::cout << "[MIXR-1] Open-Loop PWM Target: " << target_pwm_pct 
                          << "% (Scaled: " << scaled_pwm << "/4095)\n";
                updated = true;
            } catch (...) {}
        } else if (cpr_pos != std::string::npos) {
            try {
                Config::ENCODER_CPR = std::stod(line.substr(cpr_pos + 8));
                std::cout << "[MIXR-1] Live Config Update: ENCODER_CPR = " << Config::ENCODER_CPR << '\n';
            } catch (...) {}
        } else if (win_pos != std::string::npos) {
            try {
                Config::RPM_SAMPLE_WINDOW_US = std::stoi(line.substr(win_pos + 8));
                std::cout << "[MIXR-1] Live Config Update: RPM_SAMPLE_WINDOW_US = " << Config::RPM_SAMPLE_WINDOW_US << "us\n";
            } catch (...) {}
        }
    }
    return updated;
}

void TelemetryServer::stop_server() {
    disconnect_client();
    if (server_fd >= 0) close(server_fd);
    server_fd = -1;
}
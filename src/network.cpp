// src/network.cpp
#include "network.hpp"
#include "config.hpp"
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <unistd.h>
#include <atomic>
#include <algorithm>
#include <cerrno>
#include <cstring>

extern std::atomic<bool> run_loop;

void TelemetryServer::disconnect_client() {
    if (client_socket >= 0) close(client_socket);
    client_socket = -1;
    ext_requested = false;
    power_requested = false;
    send_fail_streak = 0;
    rx_buffer.clear();
}

// A dashboard that sleeps, loses Wi-Fi or is killed leaves a half-dead TCP connection. Without these settings the daemon never notices,
// keeps the old session forever and never serves the restarted dashboard (and a full send buffer would eventually block the 100 Hz loop).
void TelemetryServer::adopt_client(int fd) {
    int flag = 1;
    setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));
    setsockopt(fd, SOL_SOCKET, SO_KEEPALIVE, &flag, sizeof(flag));
    int idle = 5, interval = 2, count = 3;                      // a silent peer is declared dead after about 5 + 2 x 3 = 11 s
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPIDLE, &idle, sizeof(idle));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPINTVL, &interval, sizeof(interval));
    setsockopt(fd, IPPROTO_TCP, TCP_KEEPCNT, &count, sizeof(count));
    unsigned int user_timeout_ms = 10000;                       // unacknowledged data for 10 s -> the connection is dropped
    setsockopt(fd, IPPROTO_TCP, TCP_USER_TIMEOUT, &user_timeout_ms, sizeof(user_timeout_ms));
    int sndbuf = 16384;                                         // small buffer: a peer that stops reading is noticed within seconds
    setsockopt(fd, SOL_SOCKET, SO_SNDBUF, &sndbuf, sizeof(sndbuf));
    rx_buffer.clear();
    ext_requested = false;
    power_requested = false;
    send_fail_streak = 0;
}

// Never blocks the control loop. A peer that is gone or not reading is dropped (the main loop then falls back to standalone, motor target 0).
bool TelemetryServer::send_to_client(const std::string& packet) {
    if (client_socket < 0) return false;
    const ssize_t n = send(client_socket, packet.c_str(), packet.length(), MSG_NOSIGNAL | MSG_DONTWAIT);
    if (n == static_cast<ssize_t>(packet.length())) { send_fail_streak = 0; return true; }
    if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
        std::cout << "[MIXR-1] Dashboard connection lost (" << std::strerror(errno) << "): dropped. Motor falls back to standalone (0).\n";
        disconnect_client();
        return false;
    }
    if (++send_fail_streak > 300) {                             // about 3 s of a peer that does not read
        std::cout << "[MIXR-1] Dashboard stopped reading for 3 s: dropped. Motor falls back to standalone (0).\n";
        disconnect_client();
    }
    return false;
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
                if (client_socket >= 0) adopt_client(client_socket);
                return client_socket >= 0;
            }
        }
    }
    return false;
}

bool TelemetryServer::poll_for_client() {
    fd_set readfds;
    FD_ZERO(&readfds);
    FD_SET(server_fd, &readfds);
    struct timeval tv{0, 0};
    const bool pending = (select(server_fd + 1, &readfds, nullptr, nullptr, &tv) > 0 && FD_ISSET(server_fd, &readfds));
    if (client_socket >= 0) {
        if (pending) {
            // A NEW dashboard is knocking while an old session is still registered: the old one is stale (the dashboard was restarted,
            // the PC slept, the Wi-Fi dropped). Drop it now; the new connection is accepted on the next loop, as a clean new session.
            std::cout << "[MIXR-1] New dashboard connection: dropping the previous session.\n";
            disconnect_client();
            return false;
        }
        return true;
    }
    if (pending) {
        client_socket = accept(server_fd, nullptr, nullptr);
        if (client_socket >= 0) {
            adopt_client(client_socket);
            std::cout << "[MIXR-1] Dashboard Connected.\n";
            return true;
        }
    }
    return false;
}

bool TelemetryServer::has_client() const {
    return client_socket >= 0;
}

bool TelemetryServer::send_packet(double raw_rpm, double filtered_rpm, long long revolutions) {
    if (client_socket < 0) return false;
    std::string packet = std::to_string(raw_rpm) + "," + std::to_string(filtered_rpm) + "," + std::to_string(revolutions) + "\n";
    return send_to_client(packet);
}

bool TelemetryServer::send_packet_ext(double raw_rpm, double feedback_rpm, long long revolutions,
                                      double pwm_pct, double target_rpm, double elapsed_s,
                                      const PowerSample& power) {
    if (client_socket < 0) return false;
    std::string packet = std::to_string(raw_rpm) + "," + std::to_string(feedback_rpm) + "," +
                         std::to_string(revolutions) + "," + std::to_string(pwm_pct) + "," +
                         std::to_string(target_rpm) + "," + std::to_string(elapsed_s);
    if (power_requested) {                                      // std::to_string(NaN) gives "nan", which the dashboard reads as "no data"
        packet += "," + std::to_string(power.bus_v) + "," + std::to_string(power.current_ma) + "," + std::to_string(power.power_w);
    }
    packet += "\n";
    return send_to_client(packet);
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
        } else if (bytes == 0) {
            std::cout << "[MIXR-1] Dashboard disconnected.\n";
            disconnect_client();
            return updated;
        } else {
            if (errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR) {
                std::cout << "[MIXR-1] Dashboard connection error (" << std::strerror(errno) << "): dropped.\n";
                disconnect_client();
                return updated;
            }
            break;
        }
    }
    if (rx_buffer.size() > 4096) rx_buffer.clear();                // garbage without a newline must not grow forever

    size_t pos;
    while ((pos = rx_buffer.find('\n')) != std::string::npos) {
        std::string line = rx_buffer.substr(0, pos);
        rx_buffer.erase(0, pos + 1);

        size_t rpm_pos = line.find("CMD:RPM,");
        size_t pwm_pos = line.find("CMD:PWM,");
        size_t cpr_pos = line.find("CMD:CPR,");
        size_t win_pos = line.find("CMD:WIN,");
        size_t kp_pos  = line.find("CMD:KP,");
        size_t ki_pos  = line.find("CMD:KI,");
        size_t fa_pos  = line.find("CMD:ALPHA,");
        size_t ext_pos = line.find("CMD:EXT,");
        size_t pow_pos = line.find("CMD:POWER,");

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
        } else if (kp_pos != std::string::npos) {
            try {
                const double v = std::stod(line.substr(kp_pos + 7));
                if (v >= 0.0) {
                    Config::GLOBAL_KP = v;
                    std::cout << "[MIXR-1] Live Config Update: Kp = " << Config::GLOBAL_KP << '\n';
                }
            } catch (...) {}
        } else if (ki_pos != std::string::npos) {
            try {
                const double v = std::stod(line.substr(ki_pos + 7));
                if (v >= 0.0) {
                    Config::GLOBAL_KI = v;
                    std::cout << "[MIXR-1] Live Config Update: Ki = " << Config::GLOBAL_KI << '\n';
                }
            } catch (...) {}
        } else if (fa_pos != std::string::npos) {
            try {
                const double v = std::stod(line.substr(fa_pos + 10));
                Config::FEEDBACK_ALPHA = std::clamp(v, 0.01, 1.0);
                std::cout << "[MIXR-1] Live Config Update: FEEDBACK_ALPHA = " << Config::FEEDBACK_ALPHA << '\n';
            } catch (...) {}
        } else if (ext_pos != std::string::npos) {
            try {
                ext_requested = (std::stoi(line.substr(ext_pos + 8)) != 0);
                std::cout << "[MIXR-1] Extended telemetry " << (ext_requested ? "ON" : "OFF") << '\n';
            } catch (...) {}
        } else if (pow_pos != std::string::npos) {
            try {
                power_requested = (std::stoi(line.substr(pow_pos + 10)) != 0);
                std::cout << "[MIXR-1] Wattmeter telemetry " << (power_requested ? "ON" : "OFF") << '\n';
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
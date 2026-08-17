// include/network.hpp
#pragma once
#include <string>

class TelemetryServer {
private:
    int server_fd = -1;
    int client_socket = -1;
    std::string rx_buffer;

    void disconnect_client();

public:
    ~TelemetryServer();
    [[nodiscard]] bool start_server(int port);
    [[nodiscard]] bool wait_for_client();
    [[nodiscard]] bool send_packet(double raw_rpm, double filtered_rpm, long long revolutions) const;
    [[nodiscard]] bool receive_command(double& target_rpm);
    void stop_server();
};
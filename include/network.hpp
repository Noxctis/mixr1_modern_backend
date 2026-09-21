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
    bool start_server(int port);
    bool wait_for_client();
    
    // Updated signature to accept current and power
    bool send_packet(double raw_rpm, double filtered_rpm, long long revolutions, double current_a, double power_w) const;
    
    bool receive_command(double& target_rpm, int& target_pwm_pct, bool& pi_mode);
    
    void stop_server();
};
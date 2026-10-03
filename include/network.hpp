// include/network.hpp
// Update the receive_command signature to accept PWM and mode flags.
#pragma once
#include <string>

class TelemetryServer {
private:
    int server_fd = -1;
    int client_socket = -1;
    std::string rx_buffer;
    bool ext_requested = false;   // client sent CMD:EXT,1 -> wants the 6-field extended packet

    void disconnect_client();

public:
    ~TelemetryServer();
    bool start_server(int port);
    bool wait_for_client();
    bool poll_for_client();
    bool has_client() const;
    bool wants_ext() const { return ext_requested; }
    bool send_packet(double raw_rpm, double filtered_rpm, long long revolutions) const;

    // Extended 6-field packet used by test mode with --ext (for tools/live_plot.py):
    //   raw_rpm,feedback_rpm,revolutions,pwm_pct,target_rpm,elapsed_s
    bool send_packet_ext(double raw_rpm, double feedback_rpm, long long revolutions,
                         double pwm_pct, double target_rpm, double elapsed_s) const;
    
    // Updated signature for dual-mode control
    bool receive_command(double& target_rpm, int& target_pwm_pct, bool& pi_mode);
    
    void stop_server();
};
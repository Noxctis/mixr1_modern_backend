// include/network.hpp
// Update the receive_command signature to accept PWM and mode flags.
#pragma once
#include <string>
#include <limits>

// One wattmeter reading for the dashboard. NaN = no reading (no wattmeter, or an I2C dropout).
struct PowerSample {
    double bus_v = std::numeric_limits<double>::quiet_NaN();
    double current_ma = std::numeric_limits<double>::quiet_NaN();
    double power_w = std::numeric_limits<double>::quiet_NaN();
};

class TelemetryServer {
private:
    int server_fd = -1;
    int client_socket = -1;
    std::string rx_buffer;
    bool ext_requested = false;   // client sent CMD:EXT,1 -> wants the 6-field extended packet
    bool power_requested = false; // client sent CMD:POWER,1 -> wants 3 extra fields (bus V, current mA, power W) after the 6 extended ones
    int send_fail_streak = 0;     // consecutive sends the peer did not take (a stalled / sleeping dashboard)

    void disconnect_client();
    void adopt_client(int fd);                       // tune a freshly accepted socket (keepalive, small send buffer)
    bool send_to_client(const std::string& packet);  // never blocks; drops a dead or stalled client

public:
    ~TelemetryServer();
    bool start_server(int port);
    bool wait_for_client();
    bool poll_for_client();
    bool has_client() const;
    bool wants_ext() const { return ext_requested; }
    bool wants_power() const { return power_requested; }
    bool send_packet(double raw_rpm, double filtered_rpm, long long revolutions);

    // Extended 6-field packet used by test mode with --ext (for tools/live_plot.py):
    //   raw_rpm,feedback_rpm,revolutions,pwm_pct,target_rpm,elapsed_s
    // When the client asked for CMD:POWER,1 the packet gets 3 more fields (9 in total): ...,bus_v,current_ma,power_w
    // A client that did not ask keeps getting exactly 6 fields, so existing tools are not affected.
    bool send_packet_ext(double raw_rpm, double feedback_rpm, long long revolutions,
                         double pwm_pct, double target_rpm, double elapsed_s,
                         const PowerSample& power = PowerSample{});
    
    // Updated signature for dual-mode control
    bool receive_command(double& target_rpm, int& target_pwm_pct, bool& pi_mode);
    
    void drop_client() { disconnect_client(); }      // the main loop can drop the dashboard explicitly
    void stop_server();
};
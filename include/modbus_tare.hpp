// include/modbus_tare.hpp
// Zero (tare) command for the ATO-LCTR-OAR-DY610 load cell amplifier over an already-open, already-configured RS-485 serial port.
// Shared by ModbusTorque (daemon) and TorqueSensor (test mode). Call it ONLY with no load on the sensor, and BEFORE any other thread
// reads the same port (the daemon does it before it starts its torque thread).
//
//   1) function 0x10, write the value 0 to registers 0-1 (the manual's "clear display value"; verified on this hardware):
//        send  nn 10 00 00 00 02 04 00 00 00 00 crc0 crc1        (slave 1: 01 10 00 00 00 02 04 00 00 00 00 F3 AF)
//        reply nn 10 00 00 00 02 crc0 crc1                        (slave 1: 01 10 00 00 00 02 41 C8)
//   2) fallback, function 0x05 "clear" (same effect per the protocol sheet):
//        send  nn 05 00 00 FF 00 crc0 crc1                        (slave 1: 01 05 00 00 FF 00 8C 3A), reply = the same 8 bytes
#pragma once
#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <string>
#include <sys/select.h>
#include <sys/time.h>
#include <termios.h>
#include <unistd.h>

namespace modbus_tare {

inline uint16_t crc16(const uint8_t* d, size_t n) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < n; ++i) {
        crc ^= d[i];
        for (int b = 0; b < 8; ++b) crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
    }
    return crc;
}

// Write one request, collect exactly resp_len bytes within timeout_ms. False on a short / missing reply.
inline bool exchange(int fd, const uint8_t* req, size_t req_len, uint8_t* resp, size_t resp_len, int timeout_ms) {
    usleep(30000);                                     // more than 3.5 character times of bus silence before a Modbus frame
    tcflush(fd, TCIOFLUSH);
    if (::write(fd, req, req_len) != static_cast<ssize_t>(req_len)) return false;
    tcdrain(fd);
    size_t got = 0;
    struct timeval t0;
    gettimeofday(&t0, nullptr);
    const long limit_us = static_cast<long>(timeout_ms) * 1000L;
    while (got < resp_len) {
        struct timeval now;
        gettimeofday(&now, nullptr);
        const long used_us = (now.tv_sec - t0.tv_sec) * 1000000L + (now.tv_usec - t0.tv_usec);
        if (used_us >= limit_us) break;
        fd_set rf;
        FD_ZERO(&rf);
        FD_SET(fd, &rf);
        const long left = limit_us - used_us;
        struct timeval tv{left / 1000000L, static_cast<suseconds_t>(left % 1000000L)};
        if (select(fd + 1, &rf, nullptr, nullptr, &tv) <= 0) break;
        const ssize_t r = ::read(fd, resp + got, resp_len - got);
        if (r <= 0) break;
        got += static_cast<size_t>(r);
    }
    return got == resp_len;
}

// True when the amplifier acknowledged the zero command (the echo / reply is checked byte by byte, CRC included).
// It does NOT prove the reading is zero: the caller must read the value back after the filter has settled.
inline bool send_zero(int fd, int slave, int timeout_ms, std::string& err) {
    if (fd < 0) { err = "port not open"; return false; }
    const uint8_t id = static_cast<uint8_t>(slave);

    uint8_t w[13] = {id, 0x10, 0x00, 0x00, 0x00, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0, 0};
    uint16_t c = crc16(w, 11);
    w[11] = static_cast<uint8_t>(c & 0xFF);
    w[12] = static_cast<uint8_t>(c >> 8);
    uint8_t expect[8] = {id, 0x10, 0x00, 0x00, 0x00, 0x02, 0, 0};
    c = crc16(expect, 6);
    expect[6] = static_cast<uint8_t>(c & 0xFF);
    expect[7] = static_cast<uint8_t>(c >> 8);
    uint8_t r[8];
    if (exchange(fd, w, sizeof(w), r, sizeof(r), timeout_ms) && std::memcmp(r, expect, sizeof(r)) == 0) return true;

    uint8_t k[8] = {id, 0x05, 0x00, 0x00, 0xFF, 0x00, 0, 0};
    c = crc16(k, 6);
    k[6] = static_cast<uint8_t>(c & 0xFF);
    k[7] = static_cast<uint8_t>(c >> 8);
    if (exchange(fd, k, sizeof(k), r, sizeof(r), timeout_ms) && std::memcmp(r, k, sizeof(r)) == 0) return true;

    err = "no valid reply to the zero command (tried function 0x10, then 0x05): check slave ID, baud rate, wiring";
    return false;
}

}  // namespace modbus_tare
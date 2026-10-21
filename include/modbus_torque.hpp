// include/modbus_torque.hpp
// Self-contained Modbus-RTU (function 0x03) reader for the ATO DY610 load cell amplifier. Same serial setup as read_loadcell.py / rs485_probe.py,
// which are known to work with this amplifier: 8 data bits, no parity, 1 stop bit, raw mode. Every reply is checked byte by byte
// (slave ID, function, byte count, CRC). Header only (termios + POSIX), no pigpio and no build changes needed.
// Used by the daemon's normal mode (read) and by tools/read_loadcell.cpp (read_regs + decode).
#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>
#include <limits>
#include <cerrno>
#include <sys/select.h>
#include <sys/time.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

class ModbusTorque {
public:
    struct Settings {
        std::string port = "/dev/ttyUSB0";
        int baud = 9600;
        int slave_id = 1;
        unsigned reg_addr = 0;
        unsigned reg_count = 2;                 // 2 = signed 32-bit net weight (what the amplifier really sends), 1 = 16-bit
        std::string value_format = "int32";     // int16 | uint16 | int32 | uint32 | float32
        std::string word_order = "big";         // 32-bit values: big = first register holds the high word (the DY610 default) | little
        double scale = 0.001;                   // decoded number -> N.m (0.001 = the amplifier shows 3 decimals)
        double offset = 0.0;                    // added after scaling
        int stop_bits = 1;
        int timeout_ms = 200;                   // how long to wait for one complete reply
    };

    // probe = true: after opening, try a few reads so that available() says whether the amplifier really answers (used by the daemon).
    // probe = false: only open the port (like pymodbus connect()); opened() tells whether that worked.
    explicit ModbusTorque(const Settings& s, bool probe = true) : cfg_(s) {
        if (!open_port()) return;
        if (!probe) return;
        // Right after the port opens a USB adapter may still be settling: a few tries before giving up.
        for (int i = 0; i < 4 && !ok_; ++i) {
            double v = 0.0;
            if (read(v)) ok_ = true;
            else usleep(150000);
        }
    }
    ~ModbusTorque() { if (fd_ >= 0) close(fd_); }
    ModbusTorque(const ModbusTorque&) = delete;
    ModbusTorque& operator=(const ModbusTorque&) = delete;

    bool opened() const { return fd_ >= 0; }
    bool available() const { return ok_; }
    const std::string& last_error() const { return err_; }
    int32_t last_raw() const { return last_raw_; }

    // One raw read: the registers exactly as the amplifier sent them. Returns false on timeout / bad reply.
    bool read_regs(std::vector<uint16_t>& regs) {
        if (fd_ < 0) { err_ = "port not open"; return false; }
        const unsigned n = cfg_.reg_count == 1 ? 1 : 2;
        uint8_t req[8] = {static_cast<uint8_t>(cfg_.slave_id), 0x03, static_cast<uint8_t>(cfg_.reg_addr >> 8), static_cast<uint8_t>(cfg_.reg_addr & 0xFF),
                          0x00, static_cast<uint8_t>(n), 0, 0};
        const uint16_t c = crc16(req, 6);
        req[6] = static_cast<uint8_t>(c & 0xFF);
        req[7] = static_cast<uint8_t>(c >> 8);
        tcflush(fd_, TCIOFLUSH);
        if (::write(fd_, req, sizeof(req)) != static_cast<ssize_t>(sizeof(req))) { err_ = std::string("write failed: ") + std::strerror(errno); return false; }
        tcdrain(fd_);

        const size_t want = 5 + 2 * n;
        uint8_t buf[16];
        size_t got = 0;
        struct timeval t0;
        gettimeofday(&t0, nullptr);
        const long limit_us = static_cast<long>(cfg_.timeout_ms) * 1000L;
        while (got < want) {
            struct timeval now;
            gettimeofday(&now, nullptr);
            const long used_us = (now.tv_sec - t0.tv_sec) * 1000000L + (now.tv_usec - t0.tv_usec);
            if (used_us > limit_us) break;
            fd_set rf;
            FD_ZERO(&rf);
            FD_SET(fd_, &rf);
            const long left = limit_us - used_us;
            struct timeval tv{left / 1000000L, static_cast<suseconds_t>(left % 1000000L)};
            if (select(fd_ + 1, &rf, nullptr, nullptr, &tv) <= 0) break;
            const ssize_t r = ::read(fd_, buf + got, std::min<size_t>(want - got, sizeof(buf) - got));
            if (r <= 0) break;
            got += static_cast<size_t>(r);
        }
        if (got != want) { err_ = "no/short reply (" + std::to_string(got) + " of " + std::to_string(want) + " bytes" + hex(buf, got) + ")"; return false; }
        if (buf[0] != cfg_.slave_id || buf[1] != 0x03 || buf[2] != 2 * n) { err_ = "unexpected reply header" + hex(buf, got); return false; }
        const uint16_t cc = crc16(buf, want - 2);
        if (buf[want - 2] != (cc & 0xFF) || buf[want - 1] != (cc >> 8)) { err_ = "CRC error" + hex(buf, got); return false; }
        regs.assign(n, 0);
        for (unsigned i = 0; i < n; ++i) regs[i] = static_cast<uint16_t>((buf[3 + 2 * i] << 8) | buf[4 + 2 * i]);
        return true;
    }

    // Registers -> number, the same rules as decode() in read_loadcell.py. NaN when the format needs 2 registers and only 1 was read.
    static double decode(const std::vector<uint16_t>& regs, const std::string& format, const std::string& word_order) {
        const double nan = std::numeric_limits<double>::quiet_NaN();
        if (regs.empty()) return nan;
        if (format == "int16" || format == "uint16") {
            const uint16_t v = regs[0];
            return (format == "int16") ? static_cast<double>(static_cast<int16_t>(v)) : static_cast<double>(v);
        }
        if (regs.size() < 2) return nan;
        const uint16_t hi = word_order == "big" ? regs[0] : regs[1];
        const uint16_t lo = word_order == "big" ? regs[1] : regs[0];
        const uint32_t u = (static_cast<uint32_t>(hi) << 16) | lo;
        if (format == "uint32") return static_cast<double>(u);
        if (format == "int32") return static_cast<double>(static_cast<int32_t>(u));
        if (format == "float32") { float f; std::memcpy(&f, &u, sizeof(f)); return static_cast<double>(f); }
        return nan;
    }

    // One reading in N.m (decoded * scale + offset). Returns false on timeout / bad reply (the caller keeps its previous value).
    bool read(double& out) {
        std::vector<uint16_t> regs;
        if (!read_regs(regs)) return false;
        const double d = decode(regs, cfg_.value_format, cfg_.word_order);
        if (!std::isfinite(d)) { err_ = "cannot decode " + std::to_string(regs.size()) + " register(s) as " + cfg_.value_format + " (use --torque-regcount=2 for 32-bit formats)"; return false; }
        last_raw_ = static_cast<int32_t>(d);
        out = d * cfg_.scale + cfg_.offset;
        return true;
    }

private:
    Settings cfg_;
    int fd_ = -1;
    bool ok_ = false;
    int32_t last_raw_ = 0;
    std::string err_;

    static uint16_t crc16(const uint8_t* d, size_t n) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < n; ++i) {
            crc ^= d[i];
            for (int b = 0; b < 8; ++b) crc = (crc & 1) ? static_cast<uint16_t>((crc >> 1) ^ 0xA001) : static_cast<uint16_t>(crc >> 1);
        }
        return crc;
    }
    static std::string hex(const uint8_t* d, size_t n) {
        if (n == 0) return "";
        std::string s = ": ";
        char t[4];
        for (size_t i = 0; i < n; ++i) { std::snprintf(t, sizeof(t), "%02x ", d[i]); s += t; }
        return s;
    }
    static speed_t baud_const(int b) {
        switch (b) {
            case 2400: return B2400;
            case 4800: return B4800;
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            default: return B9600;
        }
    }
    bool open_port() {
        fd_ = ::open(cfg_.port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0) { err_ = std::string("cannot open ") + cfg_.port + ": " + std::strerror(errno); return false; }
        struct termios tio;
        if (tcgetattr(fd_, &tio) != 0) { err_ = std::string("tcgetattr: ") + std::strerror(errno); close(fd_); fd_ = -1; return false; }
        cfmakeraw(&tio);
        cfsetispeed(&tio, baud_const(cfg_.baud));
        cfsetospeed(&tio, baud_const(cfg_.baud));
        tio.c_cflag &= ~(PARENB | CSIZE | CSTOPB | CRTSCTS);
        tio.c_cflag |= CS8 | CLOCAL | CREAD;
        if (cfg_.stop_bits == 2) tio.c_cflag |= CSTOPB;
        tio.c_cc[VMIN] = 0;
        tio.c_cc[VTIME] = 0;
        if (tcsetattr(fd_, TCSANOW, &tio) != 0) { err_ = std::string("tcsetattr: ") + std::strerror(errno); close(fd_); fd_ = -1; return false; }
        tcflush(fd_, TCIOFLUSH);
        return true;
    }
};
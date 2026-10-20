// include/torquesensor.hpp
#pragma once
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>

// RS485 Modbus RTU torque sensor (read holding registers, function code 0x03), e.g. a strain-gauge
// inline torque transducer read through a USB-RS485 adapter. Mirrors the style of PowerMeter (powermeter.hpp):
// open once, available() tells you whether it's usable, read() gives one decoded+scaled value, never throws.
//
// Register layout, byte order and scale are amplifier-specific and are NOT guessed here -- set them from the
// amplifier's manual, or find them with tools/torque_calibration --probe (scans registers and prints raw values
// while you apply a known torque, so you can see which register address and width moves with the load).
class TorqueSensor {
public:
    struct Settings {
        std::string port = "/dev/ttyUSB0";
        int baud = 9600;
        int slave_id = 1;
        // Two different ATO protocols share this class. Pick the block matching your actual hardware label.
        //
        //  ATO-LCTR-OAR / DY610 load cell amplifier (confirmed from ATO's official Modbus-RTU protocol PDF for this exact
        //  model): live net weight is address 0, function 03H, 2 registers (32-bit signed, MSB-first), slave default 1,
        //  address range 1-8. Factory default serial format is "N81" = 1 stop bit (register 60 selects N81/N82). THIS IS A
        //  RAW STRAIN-GAUGE AMPLIFIER: unlike a pre-calibrated sensor, the gain and zero live in the amplifier itself and
        //  MUST be set first with real weights using its own 3 front-panel keys (SL = zero, SH = a known calibration
        //  weight) -- no amount of software scale/offset substitutes for that step. Once it's calibrated on-device, the
        //  raw register value is the weight scaled by 10^(decimal-point setting, register 68, range 0-4): scale = 10^-that.
        //  All-0xFFFF registers usually mean a fault/unset parameter, not a real reading -- re-check wiring if you see it.
        unsigned reg_addr = 0;
        unsigned reg_count = 2;
        std::string value_format = "int32";
        std::string word_order = "big";
        int stop_bits = 1;                  // DY610 factory default = N81 (1 stop bit). Set to 2 only if you changed register 60 to N82.
        double scale = 0.001;               // DY610: scale = 10^-(decimal places you set at register 68) -- confirm with a known weight, don't assume 0.001
        double offset = 0.0;                // leave at 0 -- zero the amplifier itself (SL key / address-0 clear), not in software
        std::string unit = "kg";            // whatever unit you calibrated the amplifier in (its display shows it)
        int timeout_ms = 300;               // per-request read timeout
    };

    explicit TorqueSensor(Settings settings) : cfg_(std::move(settings)) {
        fd_ = ::open(cfg_.port.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (fd_ < 0) return;                           // port does not exist / no permission: available() reports false
        fcntl(fd_, F_SETFL, 0);                         // back to blocking reads (VMIN/VTIME below control the actual timeout)
        if (!configure_port()) { ::close(fd_); fd_ = -1; return; }
        // Probe once: a real amplifier should answer its own configured register with a well-formed Modbus frame.
        std::vector<uint16_t> regs;
        ok_ = modbus_read_holding(cfg_.reg_addr, cfg_.reg_count, regs);
    }

    ~TorqueSensor() { if (fd_ >= 0) ::close(fd_); }
    TorqueSensor(const TorqueSensor&) = delete;
    TorqueSensor& operator=(const TorqueSensor&) = delete;

    bool available() const { return fd_ >= 0 && ok_; }
    const Settings& settings() const { return cfg_; }

    // One reading, decoded and scaled to `unit`. False on a comms error (caller keeps its previous value, same as PowerMeter::read).
    bool read(double& torque_out) const {
        std::vector<uint16_t> regs;
        if (!modbus_read_holding(cfg_.reg_addr, cfg_.reg_count, regs)) return false;
        torque_out = decode(regs) * cfg_.scale + cfg_.offset;
        return true;
    }

    // Raw register read at any address/count, for --probe and for registers other than the configured one.
    bool read_raw(unsigned addr, unsigned count, std::vector<uint16_t>& out) const {
        return modbus_read_holding(addr, count, out);
    }

    // Decode a register vector the same way read() does (exposed so --probe can show "what this would mean as int32" etc.).
    double decode_as(const std::vector<uint16_t>& regs, const std::string& fmt, const std::string& word_order) const {
        return decode_fmt(regs, fmt, word_order);
    }

private:
    Settings cfg_;
    int fd_ = -1;
    bool ok_ = false;

    bool configure_port() {
        termios tty{};
        if (tcgetattr(fd_, &tty) != 0) return false;
        speed_t speed = B9600;
        switch (cfg_.baud) {
            case 1200:   speed = B1200;   break;
            case 2400:   speed = B2400;   break;
            case 4800:   speed = B4800;   break;
            case 9600:   speed = B9600;   break;
            case 19200:  speed = B19200;  break;
            case 38400:  speed = B38400;  break;
            case 57600:  speed = B57600;  break;
            case 115200: speed = B115200; break;
            default:     speed = B9600;   break;
        }
        cfsetispeed(&tty, speed);
        cfsetospeed(&tty, speed);
        tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;    // 8 data bits
        tty.c_cflag &= ~PARENB;                        // no parity (Modbus RTU default 8N1; change here if your amp uses 8E1/8O1)
        if (cfg_.stop_bits >= 2) tty.c_cflag |= CSTOPB;    // 2 stop bits (the ATO DYN-series factory default)
        else                     tty.c_cflag &= ~CSTOPB;   // 1 stop bit
        tty.c_cflag &= ~CRTSCTS;                        // no hardware flow control
        tty.c_cflag |= (CREAD | CLOCAL);                // enable receiver, ignore modem control lines
        cfmakeraw(&tty);                                // raw byte I/O, no line discipline processing
        tty.c_cc[VMIN]  = 0;
        tty.c_cc[VTIME] = static_cast<cc_t>(std::max(1, cfg_.timeout_ms / 100));   // VTIME is in deciseconds
        tcflush(fd_, TCIOFLUSH);
        return tcsetattr(fd_, TCSANOW, &tty) == 0;
    }

    static uint16_t crc16_modbus(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t pos = 0; pos < len; ++pos) {
            crc ^= static_cast<uint16_t>(data[pos]);
            for (int i = 0; i < 8; ++i) {
                if (crc & 0x0001) { crc >>= 1; crc ^= 0xA001; }
                else              { crc >>= 1; }
            }
        }
        return crc;
    }

    // Function code 0x03 (read holding registers): request 8 bytes, response 5 + 2*count bytes (+ Modbus exception on error).
    bool modbus_read_holding(unsigned addr, unsigned count, std::vector<uint16_t>& out) const {
        if (fd_ < 0 || count == 0 || count > 125) return false;
        uint8_t req[8];
        req[0] = static_cast<uint8_t>(cfg_.slave_id);
        req[1] = 0x03;
        req[2] = static_cast<uint8_t>((addr >> 8) & 0xFF);
        req[3] = static_cast<uint8_t>(addr & 0xFF);
        req[4] = static_cast<uint8_t>((count >> 8) & 0xFF);
        req[5] = static_cast<uint8_t>(count & 0xFF);
        const uint16_t crc = crc16_modbus(req, 6);
        req[6] = static_cast<uint8_t>(crc & 0xFF);        // CRC low byte first on the wire
        req[7] = static_cast<uint8_t>((crc >> 8) & 0xFF);

        tcflush(fd_, TCIOFLUSH);
        if (::write(fd_, req, sizeof(req)) != static_cast<ssize_t>(sizeof(req))) return false;

        const size_t want = 5 + 2 * static_cast<size_t>(count);
        uint8_t buf[256];
        size_t have = 0;
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(cfg_.timeout_ms);
        while (have < want && std::chrono::steady_clock::now() < deadline) {
            const ssize_t n = ::read(fd_, buf + have, sizeof(buf) - have);
            if (n > 0) have += static_cast<size_t>(n);
            else if (n < 0 && errno != EAGAIN && errno != EWOULDBLOCK) return false;
        }
        if (have < 5) return false;                                       // nothing, or too little to be a real frame

        if (buf[0] != static_cast<uint8_t>(cfg_.slave_id)) return false;  // wrong / no responder
        if (buf[1] == (0x03 | 0x80)) return false;                        // Modbus exception response (illegal address/value/etc.)
        if (buf[1] != 0x03) return false;
        const uint8_t byte_count = buf[2];
        if (byte_count != 2 * count || have < static_cast<size_t>(3 + byte_count + 2)) return false;

        const uint16_t crc_calc = crc16_modbus(buf, 3 + byte_count);
        const uint16_t crc_recv = static_cast<uint16_t>(buf[3 + byte_count]) | (static_cast<uint16_t>(buf[4 + byte_count]) << 8);
        if (crc_calc != crc_recv) return false;                           // corrupted frame (noise, wrong baud, loose wire)

        out.clear();
        for (unsigned i = 0; i < count; ++i) {
            const uint16_t hi = buf[3 + 2 * i], lo = buf[4 + 2 * i];
            out.push_back(static_cast<uint16_t>((hi << 8) | lo));          // each register is big-endian on the wire, per Modbus spec
        }
        return true;
    }

    double decode(const std::vector<uint16_t>& regs) const { return decode_fmt(regs, cfg_.value_format, cfg_.word_order); }

    static double decode_fmt(const std::vector<uint16_t>& regs, const std::string& fmt, const std::string& word_order) {
        if (regs.empty()) return 0.0;
        if (fmt == "int16") {
            const auto v = static_cast<int16_t>(regs[0]);
            return static_cast<double>(v);
        }
        if (fmt == "uint16") {
            return static_cast<double>(regs[0]);
        }
        if (regs.size() < 2) return 0.0;                 // 32-bit formats need two registers
        const uint16_t hi = (word_order == "big") ? regs[0] : regs[1];
        const uint16_t lo = (word_order == "big") ? regs[1] : regs[0];
        const uint32_t u = (static_cast<uint32_t>(hi) << 16) | static_cast<uint32_t>(lo);
        if (fmt == "uint32") return static_cast<double>(u);
        if (fmt == "int32") {
            const auto v = static_cast<int32_t>(u);
            return static_cast<double>(v);
        }
        if (fmt == "float32") {
            float f;
            std::memcpy(&f, &u, sizeof(f));
            return static_cast<double>(f);
        }
        return 0.0;
    }
};
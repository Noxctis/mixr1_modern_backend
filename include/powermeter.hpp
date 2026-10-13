// include/powermeter.hpp
#pragma once
#include <cstdint>
#include <pigpiod_if2.h>

// DFRobot Gravity: I2C Digital Wattmeter (SEN0291) = TI INA219 + 10 mOhm shunt, high side, 0-26 V, +-8 A.
// Read straight from the INA219 registers (the calibration register is not needed):
//   shunt voltage  reg 0x01 : signed, 10 uV per bit   -> current = shunt voltage / R_shunt   (10 uV / 10 mOhm = 1 mA per bit)
//   bus voltage    reg 0x02 : bits 15..3, 4 mV per bit
// The chip is configured for 32-sample averaging on both channels (about 29 new readings per second), which smooths the
// 20 kHz PWM ripple so that each reading is a true average current, not a random instant of the pulse train.
// Current is positive when it flows IN+ -> IN-.  Optional gain / offset let you apply a manual calibration against a multimeter.
class PowerMeter {
private:
    int pi_;
    int handle_ = -1;
    double shunt_ohm_;
    double i_gain_;
    double i_offset_ma_;
    double v_gain_;

    // The INA219 sends the most significant byte first; SMBus word transfers are least significant byte first -> swap.
    static uint16_t swap16(uint16_t v) { return static_cast<uint16_t>((v << 8) | (v >> 8)); }

public:
    static constexpr unsigned REG_CONFIG = 0x00;
    static constexpr unsigned REG_SHUNT = 0x01;
    static constexpr unsigned REG_BUS = 0x02;
    // BRNG 32 V | PGA /4 (+-160 mV) | bus ADC 32 samples | shunt ADC 32 samples | continuous shunt + bus
    static constexpr uint16_t CONFIG = 0x36EF;

    PowerMeter(int pi, int i2c_bus, int addr, double shunt_ohm = 0.010, double i_gain = 1.0, double i_offset_ma = 0.0, double v_gain = 1.0)
        : pi_(pi), shunt_ohm_(shunt_ohm), i_gain_(i_gain), i_offset_ma_(i_offset_ma), v_gain_(v_gain) {
        handle_ = i2c_open(pi_, static_cast<unsigned>(i2c_bus), static_cast<unsigned>(addr), 0);
        if (handle_ < 0) return;
        i2c_write_word_data(pi_, static_cast<unsigned>(handle_), REG_CONFIG, swap16(CONFIG));
        const int back = i2c_read_word_data(pi_, static_cast<unsigned>(handle_), REG_CONFIG);
        if (back < 0 || swap16(static_cast<uint16_t>(back)) != CONFIG) {       // not answering like an INA219
            i2c_close(pi_, static_cast<unsigned>(handle_));
            handle_ = -1;
        }
    }

    ~PowerMeter() {
        if (handle_ >= 0) i2c_close(pi_, static_cast<unsigned>(handle_));
    }
    PowerMeter(const PowerMeter&) = delete;
    PowerMeter& operator=(const PowerMeter&) = delete;

    bool available() const { return handle_ >= 0; }

    // One reading. Returns false on an I2C error (the caller keeps its previous value).
    bool read(double& volts, double& milliamps) const {
        if (handle_ < 0) return false;
        const int rs = i2c_read_word_data(pi_, static_cast<unsigned>(handle_), REG_SHUNT);
        const int rb = i2c_read_word_data(pi_, static_cast<unsigned>(handle_), REG_BUS);
        if (rs < 0 || rb < 0) return false;
        const int16_t shunt = static_cast<int16_t>(swap16(static_cast<uint16_t>(rs)));
        const uint16_t bus = static_cast<uint16_t>(swap16(static_cast<uint16_t>(rb)) >> 3);
        milliamps = i_gain_ * (static_cast<double>(shunt) * 10.0e-6 / shunt_ohm_ * 1000.0) + i_offset_ma_;
        volts = v_gain_ * static_cast<double>(bus) * 0.004;
        return true;
    }
};
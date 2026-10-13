// include/i2c_util.hpp
#pragma once
#include <initializer_list>
#include <pigpiod_if2.h>

// Does a device ACK at this address on /dev/i2c-<bus>?  Same SMBus "quick write" probe that i2cdetect uses (harmless for the OLED and the INA219).
inline bool i2c_device_answers(int pi, int bus, int addr) {
    const int handle = i2c_open(pi, static_cast<unsigned>(bus), static_cast<unsigned>(addr), 0);
    if (handle < 0) return false;                                   // the bus does not exist, or the address is invalid
    const bool ok = (i2c_write_quick(pi, static_cast<unsigned>(handle), 0) >= 0);
    i2c_close(pi, static_cast<unsigned>(handle));
    return ok;
}

// The bus a device is on: the preferred bus first, then the others. Returns -1 when it answers on none of them.
inline int find_i2c_bus(int pi, int addr, int preferred, std::initializer_list<int> others) {
    if (i2c_device_answers(pi, preferred, addr)) return preferred;
    for (int bus : others) {
        if (bus != preferred && i2c_device_answers(pi, bus, addr)) return bus;
    }
    return -1;
}
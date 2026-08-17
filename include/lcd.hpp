// include/lcd.hpp
#pragma once
#include <string>
#include "config.hpp"

class LCD1602 {
private:
    int pi_handle;
    int i2c_handle;
    int addr;
    int backlight = 0x08;

    void write_byte(int val);
    void toggle_enable(int val);
    void send_command(int cmd);
    void send_data(int data);

public:
    LCD1602(int pi, int i2c_addr = Config::I2C_LCD_ADDR);
    ~LCD1602();
    void clear();
    void set_cursor(int row, int col);
    void print(const std::string& str);
};
// include/lcd.hpp
#pragma once
#include <string>
#include "config.hpp"

class LCD1602 {
private:
    int pi_handle;
    int i2c_handle;
    int addr;
    int cursor_row = 0;
    int cursor_col = 0;
    bool initialized = false;

    void send_command(int cmd);
    void send_data(const unsigned char* data, int len);
    void write_char(char c);

public:
    LCD1602(int pi, int i2c_addr = Config::I2C_OLED_ADDR);
    ~LCD1602();
    void clear();
    void set_cursor(int row, int col);
    void print(const std::string& str);
};
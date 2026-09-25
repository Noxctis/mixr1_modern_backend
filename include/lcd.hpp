// include/lcd.hpp
#pragma once
#include <string>
#include "config.hpp"

class LCD1602 {
private:
    enum class DriverType {
        LCD_CHAR,
        SSD1306
    };

    int pi_handle;
    int i2c_handle;
    int addr;
    DriverType driver_type = DriverType::SSD1306;
    int rows = 2;
    int cols = 16;
    bool backlight_on = true;
    int cursor_row = 0;
    int cursor_col = 0;
    bool initialized = false;

    void send_command(int cmd) const;
    void send_data(const unsigned char* data, int len) const;
    void write_char(char c);
    void send_lcd4(int value, bool rs) const;

public:
    LCD1602(int pi,
            Config::DisplayType type = Config::DISPLAY_TYPE,
            int i2c_addr = Config::I2C_OLED_ADDR);
    ~LCD1602();
    void clear();
    void set_cursor(int row, int col);
    void print(const std::string& str);
    int row_count() const { return rows; }
    int col_count() const { return cols; }
};
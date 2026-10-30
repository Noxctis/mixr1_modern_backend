// include/lcd.hpp
#pragma once
#include <string>
#include <vector>
#include <cstddef>
#include "config.hpp"

class LCD1602 {
private:
    enum class DriverType {
        LCD_CHAR,
        SSD1306,
        ST7789
    };

    int pi_handle;
    int i2c_handle = -1;
    int spi_handle = -1;
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

    // ---- ST7789 (SPI) ----
    std::vector<std::string> shadow;                      // what is on screen now, one string per text row: only changed characters are redrawn
    void tft_command(unsigned char cmd, const unsigned char* args = nullptr, int n = 0) const;
    void tft_data(const unsigned char* data, size_t len) const;
    void tft_window(int x0, int y0, int x1, int y1) const;
    bool tft_init();
    void tft_clear();
    void tft_draw_run(int row, int col, const std::string& run);   // draws a run of characters starting at (row, col)
public:
    LCD1602(int pi,
            Config::DisplayType type = Config::DISPLAY_TYPE,
            int i2c_addr = Config::I2C_OLED_ADDR,
            int i2c_bus = Config::I2C_BUS_DISPLAY);
    ~LCD1602();
    void clear();
    void set_cursor(int row, int col);
    void print(const std::string& str);
    void print_line(int row, const std::string& text);   // padded full-line write, no clear needed
    int row_count() const { return rows; }
    int col_count() const { return cols; }
};
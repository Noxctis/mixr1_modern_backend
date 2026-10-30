#include "lcd.hpp"
#include <algorithm>
#include <array>
#include <pigpiod_if2.h>
#include <unistd.h>
#include <vector>
#include <iostream>

namespace {
constexpr int OLED_WIDTH = 128;
constexpr int OLED_PAGES = 8;
constexpr int OLED_CHAR_WIDTH = 6;

// ST7789, landscape (MADCTL with MV set): 320 x 240 pixels
constexpr int TFT_WIDTH  = 320;
constexpr int TFT_HEIGHT = 240;
constexpr int TFT_COLS   = TFT_WIDTH  / (OLED_CHAR_WIDTH * Config::TFT_TEXT_SCALE);
constexpr int TFT_ROWS   = TFT_HEIGHT / (8 * Config::TFT_TEXT_SCALE);
constexpr int TFT_X_MARGIN = (TFT_WIDTH - TFT_COLS * OLED_CHAR_WIDTH * Config::TFT_TEXT_SCALE) / 2;   // centres the text block

std::array<unsigned char, 5> glyph_for(char c) {
    switch (c) {
        case '0': return {0x3E, 0x51, 0x49, 0x45, 0x3E};
        case '1': return {0x00, 0x42, 0x7F, 0x40, 0x00};
        case '2': return {0x42, 0x61, 0x51, 0x49, 0x46};
        case '3': return {0x21, 0x41, 0x45, 0x4B, 0x31};
        case '4': return {0x18, 0x14, 0x12, 0x7F, 0x10};
        case '5': return {0x27, 0x45, 0x45, 0x45, 0x39};
        case '6': return {0x3C, 0x4A, 0x49, 0x49, 0x30};
        case '7': return {0x01, 0x71, 0x09, 0x05, 0x03};
        case '8': return {0x36, 0x49, 0x49, 0x49, 0x36};
        case '9': return {0x06, 0x49, 0x49, 0x29, 0x1E};
        case 'A': return {0x7E, 0x09, 0x09, 0x09, 0x7E};
        case 'D': return {0x7F, 0x41, 0x41, 0x22, 0x1C};
        case 'E': return {0x7F, 0x49, 0x49, 0x49, 0x41};
        case 'F': return {0x7F, 0x09, 0x09, 0x09, 0x01};
        case 'L': return {0x7F, 0x40, 0x40, 0x40, 0x40};
        case 'M': return {0x7F, 0x02, 0x0C, 0x02, 0x7F};
        case 'N': return {0x7F, 0x04, 0x08, 0x10, 0x7F};
        case 'O': return {0x3E, 0x41, 0x41, 0x41, 0x3E};
        case 'P': return {0x7F, 0x09, 0x09, 0x09, 0x06};
        case 'Q': return {0x3E, 0x41, 0x51, 0x21, 0x5E};
        case 'R': return {0x7F, 0x09, 0x19, 0x29, 0x46};
        case 'S': return {0x46, 0x49, 0x49, 0x49, 0x31};
        case 'T': return {0x01, 0x01, 0x7F, 0x01, 0x01};
        case 'B': return {0x7F, 0x49, 0x49, 0x49, 0x36};
        case 'C': return {0x3E, 0x41, 0x41, 0x41, 0x22};
        case 'G': return {0x3E, 0x41, 0x49, 0x49, 0x7A};
        case 'H': return {0x7F, 0x08, 0x08, 0x08, 0x7F};
        case 'I': return {0x00, 0x41, 0x7F, 0x41, 0x00};
        case 'K': return {0x7F, 0x08, 0x14, 0x22, 0x41};
        case 'U': return {0x3F, 0x40, 0x40, 0x40, 0x3F};
        case 'V': return {0x1F, 0x20, 0x40, 0x20, 0x1F};
        case 'W': return {0x3F, 0x40, 0x38, 0x40, 0x3F};
        case 'Y': return {0x07, 0x08, 0x70, 0x08, 0x07};
        case 'Z': return {0x61, 0x51, 0x49, 0x45, 0x43};
        case '%': return {0x23, 0x13, 0x08, 0x64, 0x62};
        case 'X': return {0x63, 0x14, 0x08, 0x14, 0x63};
        case ':': return {0x00, 0x36, 0x36, 0x00, 0x00};
        case '.': return {0x00, 0x60, 0x60, 0x00, 0x00};
        case '-': return {0x08, 0x08, 0x08, 0x08, 0x08};
        case '/': return {0x20, 0x10, 0x08, 0x04, 0x02};
        case '=': return {0x14, 0x14, 0x14, 0x14, 0x14};
        case ' ': return {0x00, 0x00, 0x00, 0x00, 0x00};
        default: return {0x00, 0x00, 0x5F, 0x00, 0x00};
    }
}
}

// ============================ ST7789 (SPI) ============================
void LCD1602::tft_command(unsigned char cmd, const unsigned char* args, int n) const {
    if (spi_handle < 0) return;
    gpio_write(pi_handle, Config::PIN_TFT_DC, 0);
    spi_write(pi_handle, static_cast<unsigned>(spi_handle), reinterpret_cast<char*>(&cmd), 1);
    if (args && n > 0) {
        gpio_write(pi_handle, Config::PIN_TFT_DC, 1);
        spi_write(pi_handle, static_cast<unsigned>(spi_handle), const_cast<char*>(reinterpret_cast<const char*>(args)), static_cast<unsigned>(n));
    }
}

void LCD1602::tft_data(const unsigned char* data, size_t len) const {
    if (spi_handle < 0 || len == 0) return;
    gpio_write(pi_handle, Config::PIN_TFT_DC, 1);
    constexpr size_t CHUNK = 4096;                         // pigpiod socket transfers are kept small
    for (size_t off = 0; off < len; off += CHUNK) {
        const size_t n = std::min(CHUNK, len - off);
        spi_write(pi_handle, static_cast<unsigned>(spi_handle), const_cast<char*>(reinterpret_cast<const char*>(data + off)), static_cast<unsigned>(n));
    }
}

void LCD1602::tft_window(int x0, int y0, int x1, int y1) const {
    const unsigned char ca[4] = {static_cast<unsigned char>(x0 >> 8), static_cast<unsigned char>(x0 & 0xFF),
                                 static_cast<unsigned char>(x1 >> 8), static_cast<unsigned char>(x1 & 0xFF)};
    const unsigned char ra[4] = {static_cast<unsigned char>(y0 >> 8), static_cast<unsigned char>(y0 & 0xFF),
                                 static_cast<unsigned char>(y1 >> 8), static_cast<unsigned char>(y1 & 0xFF)};
    tft_command(0x2A, ca, 4);   // CASET
    tft_command(0x2B, ra, 4);   // RASET
    tft_command(0x2C);          // RAMWR: pixel data follows
}

bool LCD1602::tft_init() {
    using namespace Config;
    set_mode(pi_handle, PIN_TFT_DC,  PI_OUTPUT);
    set_mode(pi_handle, PIN_TFT_RES, PI_OUTPUT);
    set_mode(pi_handle, PIN_TFT_CS,  PI_OUTPUT);
    gpio_write(pi_handle, PIN_TFT_CS, 0);                  // selected for good: the ST7789 is the only device on this SPI bus
    gpio_write(pi_handle, PIN_TFT_DC, 0);

    // hardware reset
    gpio_write(pi_handle, PIN_TFT_RES, 1); usleep(10000);
    gpio_write(pi_handle, PIN_TFT_RES, 0); usleep(20000);
    gpio_write(pi_handle, PIN_TFT_RES, 1); usleep(150000);

    // bits 5 and 6 = 1: CE0 / CE1 (GPIO 8 / 7) are NOT taken over by the SPI peripheral. They belong to the extra I2C bus (bus 3).
    const unsigned flags = static_cast<unsigned>(TFT_SPI_MODE & 3) | (1u << 5) | (1u << 6);
    spi_handle = spi_open(pi_handle, static_cast<unsigned>(TFT_SPI_CHANNEL), TFT_SPI_BAUD, flags);
    if (spi_handle < 0) return false;

    tft_command(0x01); usleep(150000);                     // SWRESET
    tft_command(0x11); usleep(120000);                     // SLPOUT
    { const unsigned char a[] = {0x55};                                         tft_command(0x3A, a, 1); }   // COLMOD: 16 bit / pixel
    { const unsigned char a[] = {static_cast<unsigned char>(TFT_MADCTL)};       tft_command(0x36, a, 1); }   // MADCTL: orientation
    { const unsigned char a[] = {0x0C, 0x0C, 0x00, 0x33, 0x33};                 tft_command(0xB2, a, 5); }   // PORCTRL
    { const unsigned char a[] = {0x35};                                         tft_command(0xB7, a, 1); }   // GCTRL
    { const unsigned char a[] = {0x19};                                         tft_command(0xBB, a, 1); }   // VCOMS
    { const unsigned char a[] = {0x2C};                                         tft_command(0xC0, a, 1); }   // LCMCTRL
    { const unsigned char a[] = {0x01};                                         tft_command(0xC2, a, 1); }   // VDVVRHEN
    { const unsigned char a[] = {0x12};                                         tft_command(0xC3, a, 1); }   // VRHS
    { const unsigned char a[] = {0x20};                                         tft_command(0xC4, a, 1); }   // VDVS
    { const unsigned char a[] = {0x0F};                                         tft_command(0xC6, a, 1); }   // FRCTRL2
    { const unsigned char a[] = {0xA4, 0xA1};                                   tft_command(0xD0, a, 2); }   // PWCTRL1
    { const unsigned char a[] = {0xD0, 0x04, 0x0D, 0x11, 0x13, 0x2B, 0x3F, 0x54, 0x4C, 0x18, 0x0D, 0x0B, 0x1F, 0x23}; tft_command(0xE0, a, 14); }   // PVGAMCTRL
    { const unsigned char a[] = {0xD0, 0x04, 0x0C, 0x11, 0x13, 0x2C, 0x3F, 0x44, 0x51, 0x2F, 0x1F, 0x1F, 0x20, 0x23}; tft_command(0xE1, a, 14); }   // NVGAMCTRL
    tft_command(TFT_INVERT ? 0x21 : 0x20);                 // INVON / INVOFF
    tft_command(0x13);                                     // NORON
    tft_command(0x29); usleep(20000);                      // DISPON

    initialized = true;
    tft_clear();
    return true;
}

void LCD1602::tft_clear() {
    if (spi_handle < 0) return;
    const int w = TFT_WIDTH, h = TFT_HEIGHT;
    tft_window(0, 0, w - 1, h - 1);
    std::vector<unsigned char> chunk(4096);
    for (size_t i = 0; i + 1 < chunk.size(); i += 2) {
        chunk[i]     = static_cast<unsigned char>(Config::TFT_BG_RGB565 >> 8);
        chunk[i + 1] = static_cast<unsigned char>(Config::TFT_BG_RGB565 & 0xFF);
    }
    size_t remaining = static_cast<size_t>(w) * static_cast<size_t>(h) * 2;
    while (remaining > 0) {
        const size_t n = std::min(remaining, chunk.size());
        tft_data(chunk.data(), n);
        remaining -= n;
    }
    shadow.assign(static_cast<size_t>(rows), std::string(static_cast<size_t>(cols), ' '));
}

void LCD1602::tft_draw_run(int row, int col, const std::string& run) {
    if (!initialized || spi_handle < 0 || run.empty()) return;
    if (row < 0 || row >= rows || col < 0 || col >= cols) return;
    const int s  = Config::TFT_TEXT_SCALE;
    const int cw = OLED_CHAR_WIDTH * s;                    // cell width  (5 px glyph + 1 px gap, scaled)
    const int ch = 8 * s;                                  // cell height
    const std::string text = run.substr(0, static_cast<size_t>(cols - col));
    const int n = static_cast<int>(text.size());
    const int w = n * cw;
    const unsigned char fh = static_cast<unsigned char>(Config::TFT_FG_RGB565 >> 8), fl = static_cast<unsigned char>(Config::TFT_FG_RGB565 & 0xFF);
    const unsigned char bh = static_cast<unsigned char>(Config::TFT_BG_RGB565 >> 8), bl = static_cast<unsigned char>(Config::TFT_BG_RGB565 & 0xFF);

    std::vector<unsigned char> buf(static_cast<size_t>(w) * static_cast<size_t>(ch) * 2);
    for (int k = 0; k < n; ++k) {
        const auto g = glyph_for(text[static_cast<size_t>(k)]);   // 5 columns, bit 0 = top pixel (same font as the OLED)
        for (int gx = 0; gx < OLED_CHAR_WIDTH; ++gx) {
            const unsigned char bits = (gx < 5) ? g[static_cast<size_t>(gx)] : 0;
            for (int gy = 0; gy < 8; ++gy) {
                const bool on = (bits >> gy) & 1;
                for (int sy = 0; sy < s; ++sy) {
                    for (int sx = 0; sx < s; ++sx) {
                        const size_t px = static_cast<size_t>(k * cw + gx * s + sx);
                        const size_t py = static_cast<size_t>(gy * s + sy);
                        const size_t idx = (py * static_cast<size_t>(w) + px) * 2;
                        buf[idx]     = on ? fh : bh;
                        buf[idx + 1] = on ? fl : bl;
                    }
                }
            }
        }
    }
    const int x0 = TFT_X_MARGIN + col * cw;
    const int y0 = row * ch;
    tft_window(x0, y0, x0 + w - 1, y0 + ch - 1);
    tft_data(buf.data(), buf.size());
    for (int k = 0; k < n; ++k) shadow[static_cast<size_t>(row)][static_cast<size_t>(col + k)] = text[static_cast<size_t>(k)];
}

void LCD1602::send_command(int cmd) const {
    if (i2c_handle < 0 || driver_type != DriverType::SSD1306) return;
    char buf[2];
    buf[0] = 0x00;
    buf[1] = static_cast<char>(cmd);
    i2c_write_device(pi_handle, i2c_handle, buf, 2);
}

void LCD1602::send_data(const unsigned char* data, int len) const {
    if (i2c_handle < 0 || len <= 0 || driver_type != DriverType::SSD1306) return;
    std::vector<char> buf(static_cast<size_t>(len) + 1);
    buf[0] = 0x40;
    for (int i = 0; i < len; ++i) {
        buf[static_cast<size_t>(i) + 1] = static_cast<char>(data[i]);
    }
    i2c_write_device(pi_handle, i2c_handle, buf.data(), static_cast<unsigned>(buf.size()));
}

void LCD1602::send_lcd4(int value, bool rs) const {
    if (i2c_handle < 0 || driver_type != DriverType::LCD_CHAR) return;
    unsigned char data = static_cast<unsigned char>(value & 0xF0);
    if (rs) data |= 0x01;
    if (backlight_on) data |= 0x08;
    unsigned char pulse_on = data | 0x04;
    unsigned char pulse_off = data & static_cast<unsigned char>(~0x04);
    i2c_write_byte(pi_handle, i2c_handle, pulse_on);
    usleep(1);
    i2c_write_byte(pi_handle, i2c_handle, pulse_off);
    usleep(50);
}

LCD1602::LCD1602(int pi, Config::DisplayType type, int i2c_addr, int i2c_bus) : pi_handle(pi), addr(i2c_addr) {
    if (type == Config::DisplayType::LCD1602) {
        driver_type = DriverType::LCD_CHAR;
        rows = 2;
        cols = 16;
        addr = Config::I2C_LCD_ADDR;
    } else if (type == Config::DisplayType::LCD2004) {
        driver_type = DriverType::LCD_CHAR;
        rows = 4;
        cols = 20;
        addr = Config::I2C_LCD_ADDR;
    } else if (type == Config::DisplayType::ST7789_320x240) {
        driver_type = DriverType::ST7789;
        rows = TFT_ROWS;
        cols = TFT_COLS;
        addr = i2c_addr;
    } else {
        driver_type = DriverType::SSD1306;
        rows = 8;
        cols = OLED_WIDTH / OLED_CHAR_WIDTH;
        addr = i2c_addr;
    }

    if (driver_type == DriverType::ST7789) {
        if (addr == 0xFFFF) return;                        // --no-lcd: leave the display untouched, every call stays a no-op
        if (!tft_init()) {
            std::cerr << "[LCD] ST7789: spi_open failed (is pigpiod running? is kernel SPI disabled?). Running without a display.\n";
            initialized = false;
        }
        return;
    }

    i2c_handle = i2c_open(pi_handle, static_cast<unsigned>(i2c_bus), addr, 0);
    if (i2c_handle < 0) return;

    if (driver_type == DriverType::LCD_CHAR) {
        usleep(50000);
        send_lcd4(0x30, false);
        usleep(4500);
        send_lcd4(0x30, false);
        usleep(4500);
        send_lcd4(0x30, false);
        usleep(150);
        send_lcd4(0x20, false);

        send_lcd4(0x20, false);
        send_lcd4(rows > 1 ? 0x80 : 0x00, false);
        send_lcd4(0x00, false);
        send_lcd4(0xC0, false);
        send_lcd4(0x00, false);
        send_lcd4(0x10, false);
        send_lcd4(0x00, false);
        send_lcd4(0x60, false);
    } else {
        const unsigned char init_seq[] = {
            0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
            0x8D, 0x14, 0x20, 0x02, 0xA1, 0xC8, 0xDA, 0x12,
            0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
            0x2E, 0xAF
        };
        for (unsigned char cmd : init_seq) send_command(cmd);
    }

    initialized = true;
    clear();
}

LCD1602::~LCD1602() {
    if (driver_type == DriverType::ST7789) {
        if (spi_handle >= 0) {
            if (initialized) tft_clear();
            spi_close(pi_handle, static_cast<unsigned>(spi_handle));
            gpio_write(pi_handle, Config::PIN_TFT_CS, 1);
        }
        return;
    }
    if (i2c_handle >= 0) {
        if (initialized) {
            clear();
            if (driver_type == DriverType::SSD1306) send_command(0xAE);
        }
        i2c_close(pi_handle, i2c_handle);
    }
}

void LCD1602::clear() {
    if (!initialized) return;
    if (driver_type == DriverType::ST7789) {
        tft_clear();
        cursor_row = 0;
        cursor_col = 0;
        return;
    }
    if (driver_type == DriverType::SSD1306) {
        unsigned char blank[16] = {0};
        for (int page = 0; page < OLED_PAGES; ++page) {
            send_command(0xB0 + page);
            send_command(0x00);
            send_command(0x10);
            for (int block = 0; block < OLED_WIDTH / 16; ++block) {
                send_data(blank, 16);
            }
        }
    } else {
        send_lcd4(0x00, false);
        send_lcd4(0x10, false);
        usleep(2000);
    }
    set_cursor(0, 0);
}

void LCD1602::set_cursor(int row, int col) {
    if (!initialized) return;
    cursor_row = std::clamp(row, 0, rows - 1);
    cursor_col = std::clamp(col, 0, cols - 1);

    if (driver_type == DriverType::ST7789) return;         // the cursor is only a position here, nothing is sent
    if (driver_type == DriverType::SSD1306) {
        int x = cursor_col * OLED_CHAR_WIDTH;
        send_command(0xB0 + cursor_row);
        send_command(0x00 + (x & 0x0F));
        send_command(0x10 + ((x >> 4) & 0x0F));
    } else {
        static const int row_offsets_16x2[4] = {0x00, 0x40, 0x10, 0x50};
        static const int row_offsets_20x4[4] = {0x00, 0x40, 0x14, 0x54};
        const int* offsets = (cols == 20) ? row_offsets_20x4 : row_offsets_16x2;
        const int address = 0x80 + offsets[cursor_row] + cursor_col;
        send_lcd4(address & 0xF0, false);
        send_lcd4((address << 4) & 0xF0, false);
    }
}

void LCD1602::write_char(char c) {
    if (!initialized) return;
    if (driver_type == DriverType::ST7789) {
        tft_draw_run(cursor_row, cursor_col, std::string(1, c));
        ++cursor_col;
        return;
    }
    if (driver_type == DriverType::SSD1306) {
        auto glyph = glyph_for(c);
        unsigned char out[OLED_CHAR_WIDTH];
        for (int i = 0; i < 5; ++i) out[i] = glyph[static_cast<size_t>(i)];
        out[5] = 0x00;
        send_data(out, OLED_CHAR_WIDTH);
    } else {
        unsigned char value = static_cast<unsigned char>(c);
        send_lcd4(value & 0xF0, true);
        send_lcd4((value << 4) & 0xF0, true);
    }
    ++cursor_col;
}

void LCD1602::print(const std::string& str) {
    if (!initialized) return;
    for (char c : str) {
        if (cursor_col >= cols) break;
        write_char(c);
    }
}

void LCD1602::print_line(int row, const std::string& text) {
    if (!initialized) return;
    std::string s = text.substr(0, static_cast<size_t>(cols));
    s.resize(static_cast<size_t>(cols), ' ');
    if (driver_type == DriverType::ST7789) {
        // redraw only the characters that differ from what is on screen (a speed readout changes 2-3 characters per update)
        const int r = std::clamp(row, 0, rows - 1);
        int c = 0;
        while (c < cols) {
            if (s[static_cast<size_t>(c)] == shadow[static_cast<size_t>(r)][static_cast<size_t>(c)]) { ++c; continue; }
            const int start = c;
            while (c < cols && s[static_cast<size_t>(c)] != shadow[static_cast<size_t>(r)][static_cast<size_t>(c)]) ++c;
            tft_draw_run(r, start, s.substr(static_cast<size_t>(start), static_cast<size_t>(c - start)));
        }
        return;
    }
    set_cursor(row, 0);
    if (driver_type == DriverType::SSD1306) {
        std::vector<unsigned char> buf;
        buf.reserve(s.size() * OLED_CHAR_WIDTH);
        for (char c : s) {
            auto g = glyph_for(c);
            buf.insert(buf.end(), g.begin(), g.end());
            buf.push_back(0x00);
        }
        send_data(buf.data(), static_cast<int>(buf.size()));
        cursor_col = cols;
    } else {
        for (char c : s) write_char(c);
    }
}
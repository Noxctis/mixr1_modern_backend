// src/lcd.cpp
#include "lcd.hpp"
#include <algorithm>
#include <array>
#include <pigpiod_if2.h>
#include <vector>

namespace {
constexpr int OLED_WIDTH = 128;
constexpr int OLED_PAGES = 8;
constexpr int CHAR_WIDTH = 6;

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
        case 'R': return {0x7F, 0x09, 0x19, 0x29, 0x46};
        case 'X': return {0x63, 0x14, 0x08, 0x14, 0x63};
        case 'F': return {0x7F, 0x09, 0x09, 0x09, 0x01};
        case 'L': return {0x7F, 0x40, 0x40, 0x40, 0x40};
        case 'T': return {0x01, 0x01, 0x7F, 0x01, 0x01};
        case ':': return {0x00, 0x36, 0x36, 0x00, 0x00};
        case '.': return {0x00, 0x60, 0x60, 0x00, 0x00};
        case '-': return {0x08, 0x08, 0x08, 0x08, 0x08};
        case ' ': return {0x00, 0x00, 0x00, 0x00, 0x00};
        default: return {0x00, 0x00, 0x5F, 0x00, 0x00};
    }
}
}

void LCD1602::send_command(int cmd) {
    if (i2c_handle < 0) return;
    char buf[2];
    buf[0] = 0x00;
    buf[1] = static_cast<char>(cmd);
    i2c_write_device(pi_handle, i2c_handle, buf, 2);
}

void LCD1602::send_data(const unsigned char* data, int len) {
    if (i2c_handle < 0 || len <= 0) return;
    std::vector<char> buf(static_cast<size_t>(len) + 1);
    buf[0] = 0x40;
    for (int i = 0; i < len; ++i) {
        buf[static_cast<size_t>(i) + 1] = static_cast<char>(data[i]);
    }
    i2c_write_device(pi_handle, i2c_handle, buf.data(), static_cast<unsigned>(buf.size()));
}

LCD1602::LCD1602(int pi, int i2c_addr) : pi_handle(pi), addr(i2c_addr) {
    i2c_handle = i2c_open(pi_handle, 1, addr, 0);
    if (i2c_handle < 0) return;

    const unsigned char init_seq[] = {
        0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40,
        0x8D, 0x14, 0x20, 0x00, 0xA1, 0xC8, 0xDA, 0x12,
        0x81, 0xCF, 0xD9, 0xF1, 0xDB, 0x40, 0xA4, 0xA6,
        0x2E, 0xAF
    };
    for (unsigned char cmd : init_seq) send_command(cmd);
    initialized = true;
    clear();
}

LCD1602::~LCD1602() {
    if (i2c_handle >= 0) {
        if (initialized) {
            clear();
            send_command(0xAE);
        }
        i2c_close(pi_handle, i2c_handle);
    }
}

void LCD1602::clear() {
    if (!initialized) return;
    unsigned char blank[16] = {0};
    for (int page = 0; page < OLED_PAGES; ++page) {
        send_command(0xB0 + page);
        send_command(0x00);
        send_command(0x10);
        for (int block = 0; block < OLED_WIDTH / 16; ++block) {
            send_data(blank, 16);
        }
    }
    set_cursor(0, 0);
}

void LCD1602::set_cursor(int row, int col) {
    if (!initialized) return;
    cursor_row = std::clamp(row, 0, OLED_PAGES - 1);
    const int max_col = (OLED_WIDTH / CHAR_WIDTH) - 1;
    cursor_col = std::clamp(col, 0, max_col);
    int x = cursor_col * CHAR_WIDTH;
    send_command(0xB0 + cursor_row);
    send_command(0x00 + (x & 0x0F));
    send_command(0x10 + ((x >> 4) & 0x0F));
}

void LCD1602::write_char(char c) {
    if (!initialized) return;
    auto glyph = glyph_for(c);
    unsigned char out[CHAR_WIDTH];
    for (int i = 0; i < 5; ++i) out[i] = glyph[static_cast<size_t>(i)];
    out[5] = 0x00;
    send_data(out, CHAR_WIDTH);
    ++cursor_col;
}

void LCD1602::print(const std::string& str) {
    if (!initialized) return;
    const int max_col = OLED_WIDTH / CHAR_WIDTH;
    for (char c : str) {
        if (cursor_col >= max_col) break;
        write_char(c);
    }
}
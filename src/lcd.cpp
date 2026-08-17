// src/lcd.cpp
#include "lcd.hpp"
#include <pigpiod_if2.h>
#include <unistd.h>

void LCD1602::write_byte(int val) {
    i2c_write_byte(pi_handle, i2c_handle, val | backlight);
}

void LCD1602::toggle_enable(int val) {
    write_byte(val | 0x04);
    usleep(50); 
    write_byte(val & ~0x04);
    usleep(50);
}

void LCD1602::send_command(int cmd) {
    int high = cmd & 0xF0;
    int low = (cmd << 4) & 0xF0;
    write_byte(high); toggle_enable(high);
    write_byte(low); toggle_enable(low);
}

void LCD1602::send_data(int data) {
    int high = data & 0xF0;
    int low = (data << 4) & 0xF0;
    write_byte(high | 0x01); toggle_enable(high | 0x01);
    write_byte(low | 0x01); toggle_enable(low | 0x01);
}

LCD1602::LCD1602(int pi, int i2c_addr) : pi_handle(pi), addr(i2c_addr) {
    i2c_handle = i2c_open(pi_handle, 1, addr, 0);
    if (i2c_handle < 0) return;

    usleep(50000);
    for (int i = 0; i < 3; ++i) {
        write_byte(0x30); toggle_enable(0x30);
        usleep((i == 0) ? 5000 : 150);
    }

    write_byte(0x20); toggle_enable(0x20); 
    send_command(0x28); 
    send_command(0x0C); 
    send_command(0x06); 
    clear();
}

LCD1602::~LCD1602() {
    if (i2c_handle >= 0) {
        clear();
        i2c_close(pi_handle, i2c_handle);
    }
}

void LCD1602::clear() {
    send_command(0x01);
    usleep(2000); 
}

void LCD1602::set_cursor(int row, int col) {
    int row_offsets[] = { 0x00, 0x40 };
    send_command(0x80 | (col + row_offsets[row]));
}

void LCD1602::print(const std::string& str) {
    for (char c : str) send_data(c);
}
// tft_test.cpp - standalone ST7789 check: fills the screen red, green, blue, white, black.
// Build:  g++ tft_test.cpp -o tft_test -lpigpiod_if2 -lrt -pthread
// Run:    ./tft_test [spi_mode 0..3] [invert 0|1] [madctl hex, default 60]
// Stop the mixr1 daemon first (it uses the same SPI channel). pigpiod must be running.
#include <pigpiod_if2.h>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <unistd.h>

static int pi, spi;
static const unsigned DC = 16, RES = 19, CS = 25;

static void cmd(unsigned char c, const std::vector<unsigned char>& a = {}) {
    gpio_write(pi, DC, 0);
    spi_write(pi, spi, reinterpret_cast<char*>(&c), 1);
    if (!a.empty()) {
        gpio_write(pi, DC, 1);
        spi_write(pi, spi, const_cast<char*>(reinterpret_cast<const char*>(a.data())), a.size());
    }
}

static void fill(unsigned short rgb) {
    cmd(0x2A, {0, 0, 0x01, 0x3F});          // 0..319
    cmd(0x2B, {0, 0, 0x00, 0xEF});          // 0..239
    cmd(0x2C);
    gpio_write(pi, DC, 1);
    std::vector<unsigned char> chunk(4096);
    for (size_t i = 0; i + 1 < chunk.size(); i += 2) { chunk[i] = rgb >> 8; chunk[i + 1] = rgb & 0xFF; }
    for (size_t left = 320UL * 240 * 2; left > 0;) {
        size_t n = left < chunk.size() ? left : chunk.size();
        spi_write(pi, spi, reinterpret_cast<char*>(chunk.data()), n);
        left -= n;
    }
}

int main(int argc, char** argv) {
    const int mode = argc > 1 ? atoi(argv[1]) & 3 : 0;
    const bool inv = argc > 2 ? atoi(argv[2]) != 0 : true;
    const unsigned char madctl = argc > 3 ? static_cast<unsigned char>(strtol(argv[3], nullptr, 16)) : 0x60;

    pi = pigpio_start(nullptr, nullptr);
    if (pi < 0) { printf("FAIL: cannot reach pigpiod (sudo pigpiod)\n"); return 1; }
    set_mode(pi, DC, PI_OUTPUT); set_mode(pi, RES, PI_OUTPUT); set_mode(pi, CS, PI_OUTPUT);
    gpio_write(pi, CS, 0); gpio_write(pi, DC, 0);
    gpio_write(pi, RES, 1); usleep(10000); gpio_write(pi, RES, 0); usleep(20000); gpio_write(pi, RES, 1); usleep(150000);

    spi = spi_open(pi, 0, 20000000, static_cast<unsigned>(mode) | (1u << 5) | (1u << 6));
    if (spi < 0) { printf("FAIL: spi_open returned %d (kernel SPI enabled? another program using it?)\n", spi); pigpio_stop(pi); return 2; }
    printf("SPI open OK, mode %d, invert %d, MADCTL 0x%02X\n", mode, inv, madctl);

    cmd(0x01); usleep(150000);
    cmd(0x11); usleep(120000);
    cmd(0x3A, {0x55});
    cmd(0x36, {madctl});
    cmd(0xB2, {0x0C, 0x0C, 0x00, 0x33, 0x33}); cmd(0xB7, {0x35}); cmd(0xBB, {0x19}); cmd(0xC0, {0x2C});
    cmd(0xC2, {0x01}); cmd(0xC3, {0x12}); cmd(0xC4, {0x20}); cmd(0xC6, {0x0F}); cmd(0xD0, {0xA4, 0xA1});
    cmd(0xE0, {0xD0,0x04,0x0D,0x11,0x13,0x2B,0x3F,0x54,0x4C,0x18,0x0D,0x0B,0x1F,0x23});
    cmd(0xE1, {0xD0,0x04,0x0C,0x11,0x13,0x2C,0x3F,0x44,0x51,0x2F,0x1F,0x1F,0x20,0x23});
    cmd(inv ? 0x21 : 0x20); cmd(0x13); cmd(0x29); usleep(20000);

    const struct { const char* name; unsigned short c; } seq[] = {
        {"RED", 0xF800}, {"GREEN", 0x07E0}, {"BLUE", 0x001F}, {"WHITE", 0xFFFF}, {"BLACK", 0x0000}};
    for (auto& s : seq) { printf("filling %s\n", s.name); fill(s.c); sleep(2); }

    spi_close(pi, spi); gpio_write(pi, CS, 1); pigpio_stop(pi);
    printf("done\n");
    return 0;
}
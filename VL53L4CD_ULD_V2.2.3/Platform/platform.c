/**
  *
  * Raspberry Pi 4B Linux I2C Implementation for VL53L4CD
  * Replaces the empty ST placeholders with functional hardware calls.
  *
  ******************************************************************************
  */

#include "platform.h"
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>

/*
 * CHANGE: the I2C bus is now selectable. Default = /dev/i2c-3 (GPIO 8 = SDA, GPIO 7 = SCL).
 * Priority: VL53L4CD_PlatformSetBus(n)  >  environment variable MIXR_TOF_BUS=n  >  default 3.
 * All the register read / write functions below are unchanged. Messages are silent unless MIXR_TOF_VERBOSE is set.
 */
#define MIXR_TOF_DEFAULT_BUS 3

static int i2c_fd = -1;
static int g_bus_requested = -1;   /* set by VL53L4CD_PlatformSetBus */
static int g_bus_used = -1;

int VL53L4CD_PlatformSetBus(int bus)
{
    if (bus < 0 || bus > 99) return -1;
    g_bus_requested = bus;
    return 0;
}

int VL53L4CD_PlatformGetBus(void)
{
    return g_bus_used;
}

static int resolve_bus(void)
{
    if (g_bus_requested >= 0) return g_bus_requested;
    const char *e = getenv("MIXR_TOF_BUS");
    if (e && *e) {
        char *end = NULL;
        long v = strtol(e, &end, 10);
        if (end && *end == '\0' && v >= 0 && v <= 99) return (int)v;
        fprintf(stderr, "VL53L4CD platform: MIXR_TOF_BUS='%s' is not a bus number, using %d\n", e, MIXR_TOF_DEFAULT_BUS);
    }
    return MIXR_TOF_DEFAULT_BUS;
}

// Initialize the I2C bus on Raspberry Pi 4B (default I2C Bus 3, see above)
uint8_t VL53L4CD_PlatformInit(void) {
    if (i2c_fd >= 0) { close(i2c_fd); i2c_fd = -1; }   // a second call (bus search) must not leak the old handle
    g_bus_used = -1;
    int bus = resolve_bus();
    char path[32];
    snprintf(path, sizeof(path), "/dev/i2c-%d", bus);
    if ((i2c_fd = open(path, O_RDWR)) < 0) {
        if (getenv("MIXR_TOF_VERBOSE")) printf("Failed to open the i2c bus %s (%s)\n", path, strerror(errno));
        return 1;
    }
    // 0x29 is the default I2C address for the VL53L4CD
    if (ioctl(i2c_fd, I2C_SLAVE, 0x29) < 0) {
        if (getenv("MIXR_TOF_VERBOSE")) printf("Failed to acquire bus access and/or talk to slave (0x29) on %s.\n", path);
        close(i2c_fd);
        i2c_fd = -1;
        return 1;
    }
    g_bus_used = bus;
    return 0;
}

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t *value)
{
    uint8_t reg[2] = {RegisterAdress >> 8, RegisterAdress & 0xFF};
    uint8_t buf[4];
    
    if (write(i2c_fd, reg, 2) != 2) return 1;
    if (read(i2c_fd, buf, 4) != 4) return 1;
    
    *value = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return 0;
}

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t RegisterAdress, uint16_t *value)
{
    uint8_t reg[2] = {RegisterAdress >> 8, RegisterAdress & 0xFF};
    uint8_t buf[2];
    
    if (write(i2c_fd, reg, 2) != 2) return 1;
    if (read(i2c_fd, buf, 2) != 2) return 1;
    
    *value = (buf[0] << 8) | buf[1];
    return 0;
}

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t RegisterAdress, uint8_t *value)
{
    uint8_t reg[2] = {RegisterAdress >> 8, RegisterAdress & 0xFF};
    
    if (write(i2c_fd, reg, 2) != 2) return 1;
    if (read(i2c_fd, value, 1) != 1) return 1;
    
    return 0;
}

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t RegisterAdress, uint8_t value)
{
    uint8_t msg[3] = {RegisterAdress >> 8, RegisterAdress & 0xFF, value};
    
    if (write(i2c_fd, msg, 3) != 3) return 1;
    return 0;
}

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value)
{
    uint8_t msg[4] = {RegisterAdress >> 8, RegisterAdress & 0xFF, value >> 8, value & 0xFF};
    
    if (write(i2c_fd, msg, 4) != 4) return 1;
    return 0;
}

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value)
{
    uint8_t msg[6] = {RegisterAdress >> 8, RegisterAdress & 0xFF, 
                      (value >> 24) & 0xFF, (value >> 16) & 0xFF, 
                      (value >> 8) & 0xFF, value & 0xFF};
                      
    if (write(i2c_fd, msg, 6) != 6) return 1;
    return 0;
}

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs)
{
    usleep(TimeMs * 1000);
    return 0;
}
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

static int i2c_fd = -1;

// Initialize the I2C bus on Raspberry Pi 4B (I2C Bus 1)
uint8_t VL53L4CD_PlatformInit(void) {
    if ((i2c_fd = open("/dev/i2c-1", O_RDWR)) < 0) {
        printf("Failed to open the i2c bus /dev/i2c-1\n");
        return 1;
    }
    // 0x29 is the default I2C address for the VL53L4CD
    if (ioctl(i2c_fd, I2C_SLAVE, 0x29) < 0) {
        printf("Failed to acquire bus access and/or talk to slave (0x29).\n");
        return 1;
    }
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
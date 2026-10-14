/**
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */


#ifndef _PLATFORM_H_
#define _PLATFORM_H_
#pragma once

#include <stdint.h>
#include <string.h>

/**
* VL53L4CD device instance.
*/

typedef uint16_t Dev_t;

/**
 * @brief Error instance.
 */
typedef uint8_t VL53L4CD_Error;

/**
 * @brief If the macro below is defined, the device will be programmed to run
 * with I2C Fast Mode Plus (up to 1MHz). Otherwise, default max value is 400kHz.
 */

//#define VL53L4CD_I2C_FAST_MODE_PLUS


/**
 * @brief Read 32 bits through I2C.
 */

uint8_t VL53L4CD_RdDWord(Dev_t dev, uint16_t registerAddr, uint32_t *value);
/**
 * @brief Read 16 bits through I2C.
 */

uint8_t VL53L4CD_RdWord(Dev_t dev, uint16_t registerAddr, uint16_t *value);

/**
 * @brief Read 8 bits through I2C.
 */

uint8_t VL53L4CD_RdByte(Dev_t dev, uint16_t registerAddr, uint8_t *value);

/**
 * @brief Write 8 bits through I2C.
 */

uint8_t VL53L4CD_WrByte(Dev_t dev, uint16_t registerAddr, uint8_t value);

/**
 * @brief Write 16 bits through I2C.
 */

uint8_t VL53L4CD_WrWord(Dev_t dev, uint16_t RegisterAdress, uint16_t value);

/**
 * @brief Write 32 bits through I2C.
 */

uint8_t VL53L4CD_WrDWord(Dev_t dev, uint16_t RegisterAdress, uint32_t value);

/**
 * @brief Wait during N milliseconds.
 */

uint8_t VL53L4CD_WaitMs(Dev_t dev, uint32_t TimeMs);

/**
 * @brief Initialize the Raspberry Pi Linux I2C port
 */
uint8_t VL53L4CD_PlatformInit(void);

/**
 * @brief Choose the Linux I2C bus (/dev/i2c-N) that VL53L4CD_PlatformInit() opens.
 * Order of priority: this call, then the environment variable MIXR_TOF_BUS, then the default bus 3
 * (GPIO 8 = SDA, GPIO 7 = SCL, made with dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=8,i2c_gpio_scl=7).
 * Returns 0 if the number is valid.
 */
int VL53L4CD_PlatformSetBus(int bus);

/**
 * @brief The bus opened by the last successful VL53L4CD_PlatformInit(), or -1.
 */
int VL53L4CD_PlatformGetBus(void);

#endif  // _PLATFORM_H_
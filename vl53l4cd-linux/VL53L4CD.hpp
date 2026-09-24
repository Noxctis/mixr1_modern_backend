// VL53L4CD.hpp -- Raspberry Pi / Linux (i2c-dev) driver for the ST VL53L4CD,
// written to be a near drop-in replacement for a Pololu-style VL53L0X class:
//
//     VL53L4CD sensor;
//     sensor.initialize();                      // throws std::runtime_error on failure
//     sensor.setTimeout(500);
//     sensor.setMeasurementTimingBudget(200000); // microseconds, like the VL53L0X call
//     uint16_t mm = sensor.readRangeSingleMillimeters();
//     if (sensor.timeoutOccurred()) { ... }
//
// This is a port of Pololu's vl53l4cd-arduino library (which is itself based on
// ST's VL53L4CD ULD API, STSW-IMG026) with the Arduino Wire calls replaced by
// Linux i2c-dev ioctls. Register sequences and the default configuration table
// are taken unchanged from that library, so the license below applies.
//
// Behavioural differences from the VL53L0X class you are used to:
//   * The VL53L4CD has no single-shot mode. The sensor is kept ranging
//     continuously and readRangeSingleMillimeters() discards any result that was
//     sitting in the sensor since before the call, then waits for the next one,
//     so each call returns a reading taken after you asked for it (this matters
//     because your loops usleep() between samples).
//   * On timeout OR a non-valid ranging status the read functions return 65535,
//     the same "bad reading" value the VL53L0X class returns on timeout, so
//     existing checks such as `dist > 0 && dist < 2000` keep rejecting them.
//   * Timing budget range is 10..200 ms (VL53L0X code passing 200000 us is fine).
//   * I2C 7-bit address is 0x29 (the datasheet's "0x52" is the 8-bit write form).
//
// Original license (Pololu / STMicroelectronics):
//
// Most of the functionality of this library is based on the VL53L4CD ULD API
// provided provided by ST (STSW-IMG026), and some of the explanatory comments are
// quoted or paraphrased from the API source code, API user manual (UM2931), and
// VL53L4CD datasheet. Therefore, the license terms for the API source code (BSD
// 3-Clause Clear License) also apply to this derivative work, as specified below.
//
// For more information, see
//
// https://www.pololu.com/
// https://forum.pololu.com/
//
// --------------------------------------------------------------------------------
//
// Copyright (c) 2023 STMicroelectronics
// Copyright (c) 2024 Pololu Corporation
// All Rights Reserved
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted (subject to the limitations in the disclaimer below) provided that
// the following conditions are met:
//
// * Redistributions of source code must retain the above copyright notice, this
// list of conditions and the following disclaimer.
//
// * Redistributions in binary form must reproduce the above copyright notice, this
// list of conditions and the following disclaimer in the documentation and/or
// other materials provided with the distribution.
//
// * Neither the name of the copyright holder nor the names of its contributors may
// be used to endorse or promote products derived from this software without
// specific prior written permission.
//
// NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
// LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS
// IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
// ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
// ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>

#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

class VL53L4CD
{
  public:

    // register addresses from ULD VL53L4CD_api.h
    enum regAddr : uint16_t
    {
      SOFT_RESET                            = 0x0000,
      I2C_SLAVE__DEVICE_ADDRESS             = 0x0001,
      VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND = 0x0008,
      XTALK_PLANE_OFFSET_KCPS               = 0x0016,
      XTALK_X_PLANE_GRADIENT_KCPS           = 0x0018,
      XTALK_Y_PLANE_GRADIENT_KCPS           = 0x001A,
      RANGE_OFFSET_MM                       = 0x001E,
      INNER_OFFSET_MM                       = 0x0020,
      OUTER_OFFSET_MM                       = 0x0022,
      GPIO_HV_MUX__CTRL                     = 0x0030,
      GPIO__TIO_HV_STATUS                   = 0x0031,
      SYSTEM__INTERRUPT                     = 0x0046,
      RANGE_CONFIG_A                        = 0x005E,
      RANGE_CONFIG_B                        = 0x0061,
      RANGE_CONFIG__SIGMA_THRESH            = 0x0064,
      MIN_COUNT_RATE_RTN_LIMIT_MCPS         = 0x0066,
      INTERMEASUREMENT_MS                   = 0x006C,
      THRESH_HIGH                           = 0x0072,
      THRESH_LOW                            = 0x0074,
      SYSTEM__INTERRUPT_CLEAR               = 0x0086,
      SYSTEM_START                          = 0x0087,
      RESULT__RANGE_STATUS                  = 0x0089,
      RESULT__SPAD_NB                       = 0x008C,
      RESULT__SIGNAL_RATE                   = 0x008E,
      RESULT__AMBIENT_RATE                  = 0x0090,
      RESULT__SIGMA                         = 0x0092,
      RESULT__DISTANCE                      = 0x0096,
      RESULT__OSC_CALIBRATE_VAL             = 0x00DE,
      FIRMWARE__SYSTEM_STATUS               = 0x00E5,
      IDENTIFICATION__MODEL_ID              = 0x010F,
    };

    struct RangingData
    {
      uint16_t range_mm;
      uint8_t  range_status;   // 0 = valid; 255 = no update; see UM2931 for the rest
      uint8_t  number_of_spad;
      uint16_t signal_rate_kcps;
      uint16_t ambient_rate_kcps;
      uint16_t signal_per_spad_kcps;
      uint16_t ambient_per_spad_kcps;
      uint16_t sigma_mm;
    };

    static constexpr uint16_t BAD_READING = 65535;

    RangingData ranging_data{};

    explicit VL53L4CD(const std::string& i2c_device = "/dev/i2c-1",
                      uint8_t address7 = 0x29)
      : dev_(i2c_device), address_(address7) {}

    ~VL53L4CD()
    {
      if (fd_ >= 0)
      {
        if (ranging_) { stopContinuous(); }
        ::close(fd_);
      }
    }

    VL53L4CD(const VL53L4CD&) = delete;
    VL53L4CD& operator=(const VL53L4CD&) = delete;

    // ---- VL53L0X-style API ---------------------------------------------------

    // Opens the bus, checks the model ID, loads the default configuration and
    // starts continuous ranging (50 ms budget until you call
    // setMeasurementTimingBudget()). Throws std::runtime_error on any failure.
    void initialize(bool io_2v8 = true, bool fast_mode_plus = false)
    {
      if (fd_ < 0)
      {
        fd_ = ::open(dev_.c_str(), O_RDWR);
        if (fd_ < 0)
        {
          throw std::runtime_error("VL53L4CD: cannot open " + dev_ +
                                   " (is I2C enabled? are you in the i2c group?)");
        }
      }
      if (!init(io_2v8, fast_mode_plus))
      {
        throw std::runtime_error("VL53L4CD: init failed (no sensor at 0x29, wrong "
                                 "model ID, bus error, or boot timeout)");
      }
      startContinuous();
    }

    void setTimeout(uint16_t timeout_ms) { io_timeout_ = timeout_ms; }
    uint16_t getTimeout() const { return io_timeout_; }

    // Did a timeout occur in one of the read functions since the last call?
    bool timeoutOccurred()
    {
      bool tmp = did_timeout_;
      did_timeout_ = false;
      return tmp;
    }

    // Timing budget in microseconds (VL53L0X-style). Valid range 10000..200000.
    // Uses continuous back-to-back mode (inter-measurement period 0).
    bool setMeasurementTimingBudget(uint32_t budget_us)
    {
      uint32_t ms = budget_us / 1000;
      if (ms < 10 || ms > 200) { return false; }
      bool was_ranging = ranging_;
      if (was_ranging) { stopContinuous(); }
      bool ok = setRangeTiming(static_cast<uint8_t>(ms), 0);
      if (was_ranging) { startContinuous(); }
      return ok;
    }

    uint32_t getMeasurementTimingBudget() { return static_cast<uint32_t>(getTimingBudget()) * 1000; }

    // Returns a fresh distance in mm, or BAD_READING (65535) on timeout / invalid
    // status. Full ranging results of the last call are in ranging_data.
    uint16_t readRangeSingleMillimeters()
    {
      if (!ranging_) { startContinuous(); }
      clearInterrupt();   // drop any result measured before this call
      return read(true);
    }

    uint16_t readRangeContinuousMillimeters(bool blocking = true) { return read(blocking); }

    // Accept ranging status 1 and 2 (sigma / signal warnings: a distance is
    // returned but flagged low-confidence). Default false = only status 0 counts.
    void setAcceptWarnings(bool accept) { accept_warnings_ = accept; }

    uint8_t lastRangeStatus() const { return ranging_data.range_status; }

    // ---- Pololu-library API --------------------------------------------------

    // Model ID / boot check, default configuration, VHV, 50 ms / continuous.
    // Leaves ranging stopped. (initialize() calls this and then starts ranging.)
    bool init(bool io_2v8 = true, bool fast_mode_plus = false)
    {
      // check model ID and module type registers (values specified in datasheet)
      if (readReg16Bit(IDENTIFICATION__MODEL_ID) != 0xEBAA || last_status_ != 0) { return false; }

      // from VL53L4CD_SensorInit()

      // "Wait for boot"
      uint16_t saved_timeout = io_timeout_;
      if (io_timeout_ == 0) { io_timeout_ = 500; }   // never spin forever during init
      startTimeout();
      while (readReg(FIRMWARE__SYSTEM_STATUS) != 0x3 || last_status_ != 0)
      {
        if (checkTimeoutExpired()) { did_timeout_ = true; io_timeout_ = saved_timeout; return false; }
        ::usleep(1000);
      }

      // "Load default configuration"
      writeReg(0x2D, fast_mode_plus ? 0x12 : 0x00);
      writeReg(0x2E, io_2v8 ? 0x01 : 0x00);
      writeReg(0x2F, io_2v8 ? 0x01 : 0x00);

      // rest of default configuration copied from VL53L4CD_api.c (via Pololu's
      // library), starting at 0x30
      static const uint8_t VL53L4CD_DEFAULT_CONFIGURATION[] = {
      0x11, /* 0x30 : set bit 4 to 0 for active high interrupt and 1 for active low
      (bits 3:0 must be 0x1), use SetInterruptPolarity() */
      0x02, /* 0x31 : bit 1 = interrupt depending on the polarity,
      use CheckForDataReady() */
      0x00, /* 0x32 : not user-modifiable */
      0x02, /* 0x33 : not user-modifiable */
      0x08, /* 0x34 : not user-modifiable */
      0x00, /* 0x35 : not user-modifiable */
      0x08, /* 0x36 : not user-modifiable */
      0x10, /* 0x37 : not user-modifiable */
      0x01, /* 0x38 : not user-modifiable */
      0x01, /* 0x39 : not user-modifiable */
      0x00, /* 0x3a : not user-modifiable */
      0x00, /* 0x3b : not user-modifiable */
      0x00, /* 0x3c : not user-modifiable */
      0x00, /* 0x3d : not user-modifiable */
      0xff, /* 0x3e : not user-modifiable */
      0x00, /* 0x3f : not user-modifiable */
      0x0F, /* 0x40 : not user-modifiable */
      0x00, /* 0x41 : not user-modifiable */
      0x00, /* 0x42 : not user-modifiable */
      0x00, /* 0x43 : not user-modifiable */
      0x00, /* 0x44 : not user-modifiable */
      0x00, /* 0x45 : not user-modifiable */
      0x20, /* 0x46 : interrupt configuration 0->level low detection, 1-> level high,
      2-> Out of window, 3->In window, 0x20-> New sample ready , TBC */
      0x0b, /* 0x47 : not user-modifiable */
      0x00, /* 0x48 : not user-modifiable */
      0x00, /* 0x49 : not user-modifiable */
      0x02, /* 0x4a : not user-modifiable */
      0x14, /* 0x4b : not user-modifiable */
      0x21, /* 0x4c : not user-modifiable */
      0x00, /* 0x4d : not user-modifiable */
      0x00, /* 0x4e : not user-modifiable */
      0x05, /* 0x4f : not user-modifiable */
      0x00, /* 0x50 : not user-modifiable */
      0x00, /* 0x51 : not user-modifiable */
      0x00, /* 0x52 : not user-modifiable */
      0x00, /* 0x53 : not user-modifiable */
      0xc8, /* 0x54 : not user-modifiable */
      0x00, /* 0x55 : not user-modifiable */
      0x00, /* 0x56 : not user-modifiable */
      0x38, /* 0x57 : not user-modifiable */
      0xff, /* 0x58 : not user-modifiable */
      0x01, /* 0x59 : not user-modifiable */
      0x00, /* 0x5a : not user-modifiable */
      0x08, /* 0x5b : not user-modifiable */
      0x00, /* 0x5c : not user-modifiable */
      0x00, /* 0x5d : not user-modifiable */
      0x01, /* 0x5e : not user-modifiable */
      0xcc, /* 0x5f : not user-modifiable */
      0x07, /* 0x60 : not user-modifiable */
      0x01, /* 0x61 : not user-modifiable */
      0xf1, /* 0x62 : not user-modifiable */
      0x05, /* 0x63 : not user-modifiable */
      0x00, /* 0x64 : Sigma threshold MSB (mm in 14.2 format for MSB+LSB),
      use SetSigmaThreshold(), default value 90 mm  */
      0xa0, /* 0x65 : Sigma threshold LSB */
      0x00, /* 0x66 : Min count Rate MSB (MCPS in 9.7 format for MSB+LSB),
      use SetSignalThreshold() */
      0x80, /* 0x67 : Min count Rate LSB */
      0x08, /* 0x68 : not user-modifiable */
      0x38, /* 0x69 : not user-modifiable */
      0x00, /* 0x6a : not user-modifiable */
      0x00, /* 0x6b : not user-modifiable */
      0x00, /* 0x6c : Intermeasurement period MSB, 32 bits register,
      use SetIntermeasurementInMs() */
      0x00, /* 0x6d : Intermeasurement period */
      0x0f, /* 0x6e : Intermeasurement period */
      0x89, /* 0x6f : Intermeasurement period LSB */
      0x00, /* 0x70 : not user-modifiable */
      0x00, /* 0x71 : not user-modifiable */
      0x00, /* 0x72 : distance threshold high MSB (in mm, MSB+LSB),
      use SetD:tanceThreshold() */
      0x00, /* 0x73 : distance threshold high LSB */
      0x00, /* 0x74 : distance threshold low MSB ( in mm, MSB+LSB),
      use SetD:tanceThreshold() */
      0x00, /* 0x75 : distance threshold low LSB */
      0x00, /* 0x76 : not user-modifiable */
      0x01, /* 0x77 : not user-modifiable */
      0x07, /* 0x78 : not user-modifiable */
      0x05, /* 0x79 : not user-modifiable */
      0x06, /* 0x7a : not user-modifiable */
      0x06, /* 0x7b : not user-modifiable */
      0x00, /* 0x7c : not user-modifiable */
      0x00, /* 0x7d : not user-modifiable */
      0x02, /* 0x7e : not user-modifiable */
      0xc7, /* 0x7f : not user-modifiable */
      0xff, /* 0x80 : not user-modifiable */
      0x9B, /* 0x81 : not user-modifiable */
      0x00, /* 0x82 : not user-modifiable */
      0x00, /* 0x83 : not user-modifiable */
      0x00, /* 0x84 : not user-modifiable */
      0x01, /* 0x85 : not user-modifiable */
      0x00, /* 0x86 : clear interrupt, use ClearInterrupt() */
      0x00  /* 0x87 : start ranging, use StartRanging() or StopRanging(),
      If you want an automatic start after VL53L4CD_init() call,
      put 0x40 in location 0x87 */
      };

      const uint8_t block_size = 30;
      for (uint8_t start_reg = 0x30; start_reg <= 0x87; start_reg += block_size)
      {
        uint8_t buf[2 + block_size];
        size_t n = 0;
        buf[n++] = 0;          // reg high byte
        buf[n++] = start_reg;  // reg low byte
        for (uint8_t reg = start_reg; (reg < start_reg + block_size) && (reg <= 0x87); reg++)
        {
          buf[n++] = VL53L4CD_DEFAULT_CONFIGURATION[reg - 0x30];
        }
        last_status_ = i2cTransfer(buf, n, nullptr, 0) ? 0 : 1;
        if (last_status_) { io_timeout_ = saved_timeout; return false; }
      }

      // "Start VHV"
      writeReg(SYSTEM_START, 0x40);
      startTimeout();
      while (!dataReady())
      {
        if (checkTimeoutExpired()) { did_timeout_ = true; io_timeout_ = saved_timeout; return false; }
        ::usleep(1000);
      }

      clearInterrupt();
      stopContinuous();
      writeReg(VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, 0x09);
      writeReg(0x0B, 0);
      writeReg16Bit(0x24, 0x500);

      io_timeout_ = saved_timeout;

      // default to 50 ms timing budget, 0 inter-measurement period (continuous)
      return setRangeTiming(50, 0) && last_status_ == 0;
    }

    void setAddress(uint8_t new_addr)
    {
      writeReg(I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
      address_ = new_addr;
    }

    uint8_t getAddress() const { return address_; }

    bool dataReady() { return (readReg(GPIO__TIO_HV_STATUS) & 0x01) == 0; }
    void clearInterrupt() { writeReg(SYSTEM__INTERRUPT_CLEAR, 0x01); }

    void startContinuous()
    {
      if (readReg32Bit(INTERMEASUREMENT_MS) == 0) { writeReg(SYSTEM_START, 0x21); }  // continuous
      else                                        { writeReg(SYSTEM_START, 0x40); }  // autonomous
      ranging_ = true;
    }

    void stopContinuous()
    {
      writeReg(SYSTEM_START, 0x80);
      ranging_ = false;
    }

    // Returns range in mm when continuous ranging is active, or BAD_READING.
    uint16_t read(bool blocking = true)
    {
      if (blocking)
      {
        startTimeout();
        while (!dataReady())
        {
          if (checkTimeoutExpired()) { did_timeout_ = true; return BAD_READING; }
          ::usleep(1000);
        }
      }

      clearInterrupt();
      if (!readResults()) { return BAD_READING; }

      uint8_t s = ranging_data.range_status;
      bool ok = (s == 0) || (accept_warnings_ && (s == 1 || s == 2));
      return ok ? ranging_data.range_mm : BAD_READING;
    }

    // timing budget 10..200 ms; inter-measurement 0 (continuous) or > budget.
    // based on VL53L4CD_SetRangeTiming()
    bool setRangeTiming(uint8_t timing_budget_ms, uint32_t inter_measurement_ms)
    {
      if (timing_budget_ms < 10 || timing_budget_ms > 200) { return false; }
      if (inter_measurement_ms != 0 && inter_measurement_ms <= timing_budget_ms) { return false; }

      uint16_t osc_frequency = readReg16Bit(0x06);
      if (osc_frequency == 0 || last_status_ != 0) { return false; }

      uint32_t timing_budget_us = (uint32_t)timing_budget_ms * 1000;
      uint32_t macro_period_us = ((uint32_t)2304 * (0x40000000 / osc_frequency)) >> 6;

      if (inter_measurement_ms == 0)
      {
        // continuous mode
        writeReg32Bit(INTERMEASUREMENT_MS, 0);
        timing_budget_us -= 2500;
      }
      else
      {
        // autonomous low power mode
        uint16_t clock_pll = readReg16Bit(RESULT__OSC_CALIBRATE_VAL) & 0x3FF;
        uint32_t inter_measurement = 1.055 * inter_measurement_ms * clock_pll;
        writeReg32Bit(INTERMEASUREMENT_MS, inter_measurement);
        timing_budget_us -= 4300;
        timing_budget_us /= 2;
      }

      timing_budget_us <<= 12;

      uint32_t tmp;
      uint16_t ls_byte;
      uint8_t ms_byte;

      tmp = (macro_period_us * 16) >> 6;
      ls_byte = ((timing_budget_us + (tmp >> 1)) / tmp) - 1;
      ms_byte = 0;
      while (ls_byte > 0xFF) { ls_byte >>= 1; ms_byte++; }
      writeReg16Bit(RANGE_CONFIG_A, (uint16_t)ms_byte << 8 | ls_byte);

      tmp = (macro_period_us * 12) >> 6;
      ls_byte = ((timing_budget_us + (tmp >> 1)) / tmp) - 1;
      ms_byte = 0;
      while (ls_byte > 0xFF) { ls_byte >>= 1; ms_byte++; }
      writeReg16Bit(RANGE_CONFIG_B, (uint16_t)ms_byte << 8 | ls_byte);

      return last_status_ == 0;
    }

    // based on VL53L4CD_GetRangeTiming()
    uint8_t getTimingBudget()
    {
      uint16_t osc_frequency = readReg16Bit(0x06);
      if (osc_frequency == 0) { return 0; }
      uint16_t range_config_macrop_high = readReg16Bit(RANGE_CONFIG_A);
      uint32_t macro_period_us = ((uint32_t)2304 * (0x40000000 / osc_frequency)) >> 6;
      uint16_t ls_byte = (range_config_macrop_high & 0xFF) << 4;
      uint8_t ms_byte = (range_config_macrop_high & 0xFF00) >> 8;

      uint32_t tmp = (macro_period_us * 16) >> 6;
      uint32_t timing_budget_us = (((uint32_t)ls_byte + 1) * tmp - (tmp >> 1)) >> (16 - ms_byte);

      if (readReg32Bit(INTERMEASUREMENT_MS) == 0) { timing_budget_us += 2500; }
      else { timing_budget_us *= 2; timing_budget_us += 4300; }

      return timing_budget_us / 1000;
    }

    // ---- raw register access (16-bit register index, big-endian values) -------

    void writeReg(uint16_t reg, uint8_t value)
    {
      uint8_t b[3] = { (uint8_t)(reg >> 8), (uint8_t)reg, value };
      last_status_ = i2cTransfer(b, sizeof b, nullptr, 0) ? 0 : 1;
    }

    void writeReg16Bit(uint16_t reg, uint16_t value)
    {
      uint8_t b[4] = { (uint8_t)(reg >> 8), (uint8_t)reg, (uint8_t)(value >> 8), (uint8_t)value };
      last_status_ = i2cTransfer(b, sizeof b, nullptr, 0) ? 0 : 1;
    }

    void writeReg32Bit(uint16_t reg, uint32_t value)
    {
      uint8_t b[6] = { (uint8_t)(reg >> 8), (uint8_t)reg,
                       (uint8_t)(value >> 24), (uint8_t)(value >> 16),
                       (uint8_t)(value >> 8),  (uint8_t)value };
      last_status_ = i2cTransfer(b, sizeof b, nullptr, 0) ? 0 : 1;
    }

    uint8_t readReg(uint16_t reg)
    {
      uint8_t w[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
      uint8_t r[1] = { 0 };
      last_status_ = i2cTransfer(w, 2, r, 1) ? 0 : 1;
      return r[0];
    }

    uint16_t readReg16Bit(uint16_t reg)
    {
      uint8_t w[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
      uint8_t r[2] = { 0, 0 };
      last_status_ = i2cTransfer(w, 2, r, 2) ? 0 : 1;
      return (uint16_t)r[0] << 8 | r[1];
    }

    uint32_t readReg32Bit(uint16_t reg)
    {
      uint8_t w[2] = { (uint8_t)(reg >> 8), (uint8_t)reg };
      uint8_t r[4] = { 0, 0, 0, 0 };
      last_status_ = i2cTransfer(w, 2, r, 4) ? 0 : 1;
      return (uint32_t)r[0] << 24 | (uint32_t)r[1] << 16 | (uint32_t)r[2] << 8 | r[3];
    }

    int lastBusStatus() const { return last_status_; }   // 0 = last I2C transfer OK

  private:

    std::string dev_;
    int      fd_ = -1;
    uint8_t  address_;
    int      last_status_ = 0;
    bool     ranging_ = false;
    bool     accept_warnings_ = false;

    uint16_t io_timeout_ = 500;   // ms; 0 = wait forever (not recommended near a pump)
    bool     did_timeout_ = false;
    std::chrono::steady_clock::time_point timeout_start_;

    void startTimeout() { timeout_start_ = std::chrono::steady_clock::now(); }
    bool checkTimeoutExpired() const
    {
      if (io_timeout_ == 0) { return false; }
      auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                       std::chrono::steady_clock::now() - timeout_start_).count();
      return elapsed > io_timeout_;
    }

    // One I2C write (wlen bytes) optionally followed by a repeated-start read
    // (rlen bytes), as a single i2c-dev transaction.
    bool i2cTransfer(const uint8_t* w, size_t wlen, uint8_t* r, size_t rlen)
    {
      if (fd_ < 0) { return false; }
      struct i2c_msg msgs[2];
      unsigned n = 0;
      msgs[n].addr  = address_;
      msgs[n].flags = 0;
      msgs[n].len   = static_cast<uint16_t>(wlen);
      msgs[n].buf   = const_cast<uint8_t*>(w);
      n++;
      if (rlen > 0)
      {
        msgs[n].addr  = address_;
        msgs[n].flags = I2C_M_RD;
        msgs[n].len   = static_cast<uint16_t>(rlen);
        msgs[n].buf   = r;
        n++;
      }
      struct i2c_rdwr_ioctl_data data;
      data.msgs  = msgs;
      data.nmsgs = n;
      return ::ioctl(fd_, I2C_RDWR, &data) == static_cast<int>(n);
    }

    // based on VL53L4CD_GetResult()
    bool readResults()
    {
      static const uint8_t status_rtn[24] = { 255, 255, 255, 5, 2, 4, 1, 7, 3,
        0, 255, 255, 9, 13, 255, 255, 255, 255, 10, 6,
        255, 255, 11, 12 };

      // Block read 15 bytes from 0x89 (RESULT__RANGE_STATUS) to 0x97 (lower byte
      // of RESULT_DISTANCE).
      uint8_t w[2] = { (uint8_t)(RESULT__RANGE_STATUS >> 8), (uint8_t)RESULT__RANGE_STATUS };
      uint8_t buffer[15] = { 0 };
      last_status_ = i2cTransfer(w, 2, buffer, 15) ? 0 : 1;
      if (last_status_ != 0)
      {
        ranging_data.range_status = 255;
        return false;
      }

      uint8_t status = buffer[0] & 0x1F; // 0x89
      if (status < 24) { status = status_rtn[status]; }
      ranging_data.range_status = status;

      ranging_data.number_of_spad      = buffer[3];                                          // 0x8C
      ranging_data.signal_rate_kcps    = ((uint16_t)buffer[5] << 8 | buffer[6]) * 8;         // 0x8E, 0x8F
      ranging_data.ambient_rate_kcps   = ((uint16_t)buffer[7] << 8 | buffer[8]) * 8;         // 0x90, 0x91
      ranging_data.sigma_mm            = ((uint16_t)buffer[9] << 8 | buffer[10]) / 4;        // 0x92, 0x93
      ranging_data.range_mm            = (uint16_t)buffer[13] << 8 | buffer[14];             // 0x96, 0x97

      // (guard added vs. the Arduino library: avoid integer divide-by-zero)
      if (ranging_data.number_of_spad != 0)
      {
        ranging_data.signal_per_spad_kcps  = ranging_data.signal_rate_kcps  / ranging_data.number_of_spad;
        ranging_data.ambient_per_spad_kcps = ranging_data.ambient_rate_kcps / ranging_data.number_of_spad;
      }
      else
      {
        ranging_data.signal_per_spad_kcps  = 0;
        ranging_data.ambient_per_spad_kcps = 0;
      }
      return true;
    }
};

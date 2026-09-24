// vl53l4cd_test.cpp -- first-power-on check for the Pololu VL53L4CD carrier.
//
//   g++ -O2 -std=c++17 vl53l4cd_test.cpp -o vl53l4cd_test
//   ./vl53l4cd_test            # uses /dev/i2c-1, address 0x29
//
// Point the sensor at a flat target 50-500 mm away and watch the numbers.
#include "VL53L4CD.hpp"
#include <cstdio>
#include <iostream>

int main()
{
    VL53L4CD sensor;   // /dev/i2c-1, 0x29
    try {
        sensor.initialize();
        sensor.setTimeout(500);
        if (!sensor.setMeasurementTimingBudget(50000)) {
            std::cerr << "Could not set timing budget\n";
            return 2;
        }
    } catch (const std::exception& e) {
        std::cerr << "Init failed: " << e.what() << "\n"
                  << "Check: `sudo i2cdetect -y 1` shows 29, VIN is on 3.3V, "
                     "protective liner removed.\n";
        return 1;
    }

    std::printf("Sensor OK. Timing budget = %u us\n\n", sensor.getMeasurementTimingBudget());
    std::printf("%4s  %8s  %6s  %8s  %10s  %6s\n",
                "#", "range_mm", "status", "sigma_mm", "sig_kcps/spad", "SPADs");

    for (int i = 1; i <= 20; ++i) {
        uint16_t mm = sensor.readRangeSingleMillimeters();
        const auto& d = sensor.ranging_data;
        if (sensor.timeoutOccurred()) {
            std::printf("%4d  TIMEOUT\n", i);
        } else if (mm == VL53L4CD::BAD_READING) {
            std::printf("%4d  invalid (status %u, raw %u mm)\n", i, d.range_status, d.range_mm);
        } else {
            std::printf("%4d  %8u  %6u  %8u  %10u  %6u\n",
                        i, mm, d.range_status, d.sigma_mm, d.signal_per_spad_kcps, d.number_of_spad);
        }
    }
    return 0;
}

#!/usr/bin/env bash
# Puts the Raspberry Pi in the same state for every run (same speed, same pigpio sampling).
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor > /dev/null
sudo killall pigpiod 2>/dev/null; sleep 1; sudo pigpiod -s 5
sleep 1
echo "governor: $(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_governor)"
echo "pigpiod:  $(ps -C pigpiod -o args= | head -1)"
echo "temp:     $(vcgencmd measure_temp 2>/dev/null)   throttled: $(vcgencmd get_throttled 2>/dev/null)  (0x0 = never throttled)"

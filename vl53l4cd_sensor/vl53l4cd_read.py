import time
import board
import adafruit_vl53l4cd

# Initialize I2C and Sensor
i2c = board.I2C()
vl53 = adafruit_vl53l4cd.VL53L4CD(i2c)

# Maximize Hardware Accuracy for Characterization
# 200ms timing budget gives the lowest hardware noise (~5Hz sample rate)
vl53.timing_budget = 200
vl53.inter_measurement = 0 

print("Starting Raw VL53L4CD Characterization...")
print("Sample, Raw_Distance_mm")

sample_count = 0
vl53.start_ranging()

try:
    while True:
        # Wait for the hardware to signal data is ready
        while not vl53.data_ready:
            pass
        
        vl53.clear_interrupt()
        
        # Read raw distance in cm and convert to mm
        raw_distance_mm = vl53.distance * 10.0
        
        sample_count += 1
        
        # Print in a CSV-friendly format
        print(f"{sample_count}, {raw_distance_mm:.2f}")
        
except KeyboardInterrupt:
    vl53.stop_ranging()
    print("\nCharacterization stopped.")

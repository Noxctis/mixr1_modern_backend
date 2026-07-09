cmake_minimum_required (VERSION 2.8.8 FATAL_ERROR)

# Examples
set (VL53L0X_EXAMPLES
	single
	singleMinimal
	continuousMultipleSensors
)

foreach (example ${VL53L0X_EXAMPLES})
	add_executable (${example}
		${example}.cpp
		../VL53L0X.cpp
		../I2Cdev.cpp
	)
	target_include_directories (${example} PUBLIC
		${CMAKE_CURRENT_SOURCE_DIR}/../
	)
endforeach (example)

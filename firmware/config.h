#ifndef CONFIG_H
#define CONFIG_H

#define BAUD_RATE 115200
#define READ_DELAY 100  // Read every 100ms

// Gravity constant
#define G 9.80665

// Sensitivity Scaling Factors (MPU9250 datasheet)
#define ACCEL_SCALE  (G / 16384.0)  // ±2g range (LSB to m/s²)
#define GYRO_SCALE   (1.0 / 32.8)   // ±1000°/s range (LSB to °/s)
#define MAG_SCALE    (0.15)         // 0.15 µT per LSB (16-bit mode)

// Temperature conversion
#define TEMP_SCALE   (1.0 / 333.87)  // LSB/°C
#define TEMP_OFFSET  21.0            // Offset in degrees Celsius

#endif

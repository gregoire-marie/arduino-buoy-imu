#ifndef CONFIG_H
#define CONFIG_H

#define BAUD_RATE 115200
#define READ_DELAY 100  // Read every 100ms

#define G 9.80665

// Sensitivity Scaling Factors (Based on MPU9250 datasheet)
#define ACCEL_SCALE  (G * 2.0 / 32768.0)  // Assuming ±2g range
#define GYRO_SCALE   (250.0 / 32768.0) // Assuming ±250°/s range
#define MAG_SCALE    (4800.0 / 32768.0) // Micro-Tesla (uT) scale

#endif

#include "mpu9250.h"

// MPU9250 I2C Address
#define MPU9250_ADDR 0x68
#define AK8963_ADDR  0x0C

// Register Addresses
#define ACCEL_XOUT_H  0x3B
#define TEMP_OUT_H    0x41
#define GYRO_XOUT_H   0x43
#define MAG_XOUT_L    0x03
#define PWR_MGMT_1    0x6B
#define WHO_AM_I      0x75
#define MAG_CONTROL   0x0A
#define MAG_ADJ_X     0x10

bool MPU9250::init() {
    // Initialize connection to the MPU9250
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(PWR_MGMT_1);
    Wire.write(0x00); // Wake up the sensor
    if (Wire.endTransmission() != 0) return false;

    // Verify connection
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, 1);
    if (Wire.available() && Wire.read() != 0x71) return false;

    // Configure Gyro and Accelerometer Ranges
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(27);
    Wire.write(0x10); // ±1000°/s
    Wire.endTransmission();

    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(28);
    Wire.write(0x00); // ±2g
    Wire.endTransmission();

    // Enable bypass mode to access magnetometer
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(0x37);
    Wire.write(0x02);
    Wire.endTransmission();

    setMagnetometerAdjustmentValues();

    // Start Magnetometer in 16-bit continuous mode
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(MAG_CONTROL);
    Wire.write(0x16); // 16-bit output, continuous mode
    Wire.endTransmission();

    return true;
}

void MPU9250::setMagnetometerAdjustmentValues() {
    uint8_t buffer[3];

    // Enter Fuse ROM Access Mode
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(MAG_CONTROL);
    Wire.write(0x1F);
    Wire.endTransmission();
    delay(100);

    // Read factory sensitivity adjustment values
    readRawData(MAG_ADJ_X, 3, buffer);

    magAdjustment.x = buffer[0];
    magAdjustment.y = buffer[1];
    magAdjustment.z = buffer[2];

    // Exit Fuse ROM Access Mode and return to normal operation
    Wire.beginTransmission(AK8963_ADDR);
    Wire.write(MAG_CONTROL);
    Wire.write(0x10);
    Wire.endTransmission();
}

void MPU9250::readSensorData() {
    uint8_t buffer[14];
    readRawData(ACCEL_XOUT_H, 14, buffer);

    // Accelerometer Data
    int16_t rawAccX = (buffer[0] << 8) | buffer[1];
    int16_t rawAccY = (buffer[2] << 8) | buffer[3];
    int16_t rawAccZ = (buffer[4] << 8) | buffer[5];

    // Temperature Data
    int16_t rawTemp = (buffer[6] << 8) | buffer[7];

    // Gyroscope Data
    int16_t rawGyroX = (buffer[8] << 8) | buffer[9];
    int16_t rawGyroY = (buffer[10] << 8) | buffer[11];
    int16_t rawGyroZ = (buffer[12] << 8) | buffer[13];

    // Magnetometer Data
    uint8_t magBuffer[6];
    readRawData(MAG_XOUT_L, 6, magBuffer);
    int16_t rawMagX = (magBuffer[1] << 8) | magBuffer[0];
    int16_t rawMagY = (magBuffer[3] << 8) | magBuffer[2];
    int16_t rawMagZ = (magBuffer[5] << 8) | magBuffer[4];

    // Assign raw values
    accX = rawAccX;
    accY = rawAccY;
    accZ = rawAccZ;
    gyroX = rawGyroX;
    gyroY = rawGyroY;
    gyroZ = rawGyroZ;
    magX = rawMagX;
    magY = rawMagY;
    magZ = rawMagZ;
    temperature = rawTemp;
}

void MPU9250::normalizeData() {
    accX *= ACCEL_SCALE;
    accY *= ACCEL_SCALE;
    accZ *= ACCEL_SCALE;

    gyroX *= GYRO_SCALE;
    gyroY *= GYRO_SCALE;
    gyroZ *= GYRO_SCALE;

    magX *= MAG_SCALE * (((magAdjustment.x - 128) / 256.0) + 1);
    magY *= MAG_SCALE * (((magAdjustment.y - 128) / 256.0) + 1);
    magZ *= MAG_SCALE * (((magAdjustment.z - 128) / 256.0) + 1);

    temperature = (temperature * TEMP_SCALE) + TEMP_OFFSET;
}

void MPU9250::readRawData(int addr, int bytes, uint8_t* buffer) {
    Wire.beginTransmission(MPU9250_ADDR);
    Wire.write(addr);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU9250_ADDR, bytes);

    for (int i = 0; i < bytes; i++) {
        if (Wire.available()) {
            buffer[i] = Wire.read();
        }
    }
}

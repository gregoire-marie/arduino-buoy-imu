#ifndef MPU9250_H
#define MPU9250_H

#include "Arduino.h"
#include "Wire.h"
#include "config.h"

class MPU9250 {
public:
    float accX, accY, accZ;
    float gyroX, gyroY, gyroZ;
    float magX, magY, magZ;
    float temperature;

    bool init();
    void readSensorData();
    void normalizeData();
    void setMagnetometerAdjustmentValues();

private:
    void readRawData(int addr, int bytes, uint8_t* buffer);

    struct {
        uint8_t x, y, z; // Factory sensitivity adjustment values
    } magAdjustment;
};

#endif

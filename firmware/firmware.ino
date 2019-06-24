#include "Wire.h"
#include "mpu9250.h"

MPU9250 mpu;

void setup() {
    Serial.begin(BAUD_RATE);
    Wire.begin();

    if (!mpu.init()) {
        Serial.println("{\"status\": \"error\", \"message\": \"MPU9250 initialization failed!\"}");
        while (1);
    }

    Serial.println("{\"status\": \"ok\", \"message\": \"MPU9250 initialized successfully!\"}");
}

void loop() {
    mpu.readSensorData();
    mpu.normalizeData();

    unsigned long timestamp = millis()

    // Send sensor data in JSON format
    Serial.print("{");
    Serial.print("\"timestamp\": "); Serial.print(timestamp); Serial.print(", ");

    Serial.print("\"temperature\": "); Serial.print(mpu.temperature, 4); Serial.print(", ");

    Serial.print("\"accelerometer\": {");
    Serial.print("\"x\": "); Serial.print(mpu.accX, 4); Serial.print(", ");
    Serial.print("\"y\": "); Serial.print(mpu.accY, 4); Serial.print(", ");
    Serial.print("\"z\": "); Serial.print(mpu.accZ, 4);
    Serial.print("}, ");

    Serial.print("\"gyroscope\": {");
    Serial.print("\"x\": "); Serial.print(mpu.gyroX, 4); Serial.print(", ");
    Serial.print("\"y\": "); Serial.print(mpu.gyroY, 4); Serial.print(", ");
    Serial.print("\"z\": "); Serial.print(mpu.gyroZ, 4);
    Serial.print("}, ");

    Serial.print("\"magnetometer\": {");
    Serial.print("\"x\": "); Serial.print(mpu.magX, 4); Serial.print(", ");
    Serial.print("\"y\": "); Serial.print(mpu.magY, 4); Serial.print(", ");
    Serial.print("\"z\": "); Serial.print(mpu.magZ, 4);
    Serial.print("}");

    Serial.println("}");

    delay(READ_DELAY);
}

#include "Wire.h"
#include "MPU9250.h"
#include "config.h"

MPU9250 mpu;
unsigned long lastPrintMillis = 0;

void setup() {
    Serial.begin(BAUD_RATE);
    Wire.begin();

    Serial.println("Initializing MPU9250...");

    MPU9250Setting setting;
    setting.accel_fs_sel = ACCEL_FS_SEL::A16G;
    setting.gyro_fs_sel = GYRO_FS_SEL::G1000DPS;
    setting.mag_output_bits = MAG_OUTPUT_BITS::M16BITS;
    setting.fifo_sample_rate = FIFO_SAMPLE_RATE::SMPL_250HZ;
    setting.gyro_fchoice = 0x03;
    setting.gyro_dlpf_cfg = GYRO_DLPF_CFG::DLPF_20HZ;
    setting.accel_fchoice = 0x01;
    setting.accel_dlpf_cfg = ACCEL_DLPF_CFG::DLPF_45HZ;

    if (!mpu.setup(0x68, setting)) {
        Serial.println("{\"status\": \"error\", \"message\": \"MPU9250 initialization failed!\"}");
        while (1);
    }

    mpu.setMagneticDeclination(MAGNETIC_DECLINATION);
    mpu.selectFilter(QuatFilterSel::MADGWICK);
    mpu.setFilterIterations(15);

    // Calibrate Accelerometer and Gyroscope
    Serial.println("Calibration will start in 5 seconds...");
    delay(5000);
    Serial.println("Calibrating Accelerometer and Gyroscope...");
    mpu.calibrateAccelGyro();

    // Calibrate Magnetometer
    Serial.println("Calibrating Magnetometer in 5 seconds...");
    delay(5000);
    Serial.println("Calibrating Magnetometer...");
    mpu.calibrateMag();

    Serial.println("{\"status\": \"ok\", \"message\": \"MPU9250 initialized successfully!\"}");
}

void loop() {
    unsigned long currentMillis = millis();

    if (mpu.update() && (currentMillis - lastPrintMillis >= READ_DELAY)) {
        Serial.print("{");

        Serial.print("\"timestamp\": ");
        Serial.print(millis());
        Serial.print(", ");

        Serial.print("\"temperature\": ");
        Serial.print(mpu.getTemperature(), 2);
        Serial.print(", ");

        Serial.print("\"accelerometer\": {");
        Serial.print("\"x\": "); Serial.print(mpu.getAccX(), 4);
        Serial.print(", \"y\": "); Serial.print(mpu.getAccY(), 4);
        Serial.print(", \"z\": "); Serial.print(mpu.getAccZ(), 4);
        Serial.print("}, ");

        Serial.print("\"gyroscope\": {");
        Serial.print("\"x\": "); Serial.print(mpu.getGyroX(), 4);
        Serial.print(", \"y\": "); Serial.print(mpu.getGyroY(), 4);
        Serial.print(", \"z\": "); Serial.print(mpu.getGyroZ(), 4);
        Serial.print("}, ");

        Serial.print("\"magnetometer\": {");
        Serial.print("\"x\": "); Serial.print(mpu.getMagX(), 4);
        Serial.print(", \"y\": "); Serial.print(mpu.getMagY(), 4);
        Serial.print(", \"z\": "); Serial.print(mpu.getMagZ(), 4);
        Serial.print("}, ");

        Serial.print("\"orientation\": {");
        Serial.print("\"pitch\": "); Serial.print(mpu.getPitch(), 4);
        Serial.print(", \"roll\": "); Serial.print(mpu.getRoll(), 4);
        Serial.print(", \"yaw\": "); Serial.print(mpu.getYaw(), 4);
        Serial.print("}");

        Serial.println("}");
        lastPrintMillis = currentMillis;
    }
}

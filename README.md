# Arduino Buoy

## Overview
A modular framework for an oceanic buoy using an **Arduino Nano** and an **MPU9250 IMU** to collect and analyze motion data for ocean swell mapping.

## Features
- **Real-time IMU Data Acquisition**: Accelerometer, gyroscope, and magnetometer readings.
- **Data Analysis**: Python and MATLAB tools for processing and visualization.
- **Position Estimation**: Kalman filtering and integration algorithms.
- **Serial Communication**: Transmit data by cable to the computer.
- **Simulation Mode**: IMU simulator for algorithm testing without hardware.

## Repository Structure
```
├── app/                        # Launchable scripts
├── docs/                       # Documentation
├── firmware/                   # Arduino Nano firmware (C++)
└── src/
    └── main/
        ├── matlab/
        └── python/
            ├── plot/           # Plotting utilities & real-time visualization
            ├── serial/         # Serial communication & data acquisition
            └── simulation/     # IMU Simulator
```

## Quick Start
### Requirements
- **Hardware**: Arduino Nano, MPU9250 IMU, serial interface
- **Software**: 
  - Python 3.10 (see requirements.txt)
  - [Arduino IDE](https://www.arduino.cc/en/software)
  - MATLAB (optional)

### Install the MPU9250 Library
1. Download the I2C library .ZIP via the [official repository](https://github.com/hibit-dev/mpu9250/raw/master/lib/I2C.zip)
2. Import to Arduino IDE via `Sketch > Include Library > Add .ZIP Library`.

### Flash the Arduino Firmware
1. **Connect the Arduino Nano** via USB.
2. **Open Arduino IDE**, go to `File > Open`.
3. Select `firmware/firmware.ino`.
4. In `Tools > Board`, select **Arduino Nano**.
5. In `Tools > Processor`, select **ATmega328P**.
   - If the upload fails, try **ATmega328P (Old Bootloader)**.
6. In `Tools > Port`, select the correct port (e.g., `COM3` on Windows or `/dev/ttyUSB0` on Linux/Mac).
7. Click the `✓ Verify` button to compile the firmware.
8. Click the `⬆ Upload` button to flash the firmware to the Arduino.
9. Open `Tools > Serial Monitor`, set **baud rate to 115200**, and check the output.

### Run the app
1. **Ensure the Arduino is connected** and runs the firmware.
2. **Run the real-time acquisition and plot script**:
   ```bash
   python app/plot_serial.py

## License
Released under the **MIT License**.
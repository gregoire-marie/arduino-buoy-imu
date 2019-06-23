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
├── data_acquisition/       # Python scripts for real-time visualization
├── docs/                   # Documentation
├── firmware/               # Arduino Nano firmware (C++)
├── matlab_analysis/        # MATLAB scripts for analysis
└── simulation/             # Simulated IMU for testing
```

## Quick Start
### Requirements
- **Hardware**: Arduino Nano, MPU9250 IMU, serial interface
- **Software**: Python (matplotlib, numpy, pyserial), MATLAB (optional)

### Steps
1. **Flash the Arduino**: Upload `imu_data_logger.ino` to the Arduino Nano.
2. **Run Data Logger**: Execute `plot_realtime.py` to visualize IMU data.
3. **Use Simulation Mode** *(Optional)*: Run `serial_simulator.py` for testing without hardware.
4. **Analyze Data**: Process collected data with Python or MATLAB.

## License
Released under the **MIT License**.
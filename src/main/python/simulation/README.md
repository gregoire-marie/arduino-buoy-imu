# Inertial Measurement Unit (IMU) - Simulator

## Functioning
We differentiate the equations of motion for real-time IMU simulation: **accelerometer**, **gyroscope**, **magnetometer**, and **thermometer**.

### Trajectories
Multiple cases are available. To add a new one, just implement a child to the class `Trajectory`. Make sure the positions and orientations start at 0, to mimic a calibration. 

The trajectories provide a set of `positions` and `orientations`, respectively in the (`x`, `y`, `z`) and (`roll`, `pitch`, `yaw`) frames.

---

## 1. Accelerometer

The **accelerometer** measures **accelerations** in m/s^2, along the X, Y and Z local axes.

They are computed by differentiating the trajectory positions, to obtain the velocities, which are differentiated a second time to get the accelerations:

`a[i] = (v[i] - v[i - 1]) / dt` 

Where: `v[i] = (x[i + 1] - x[i]) / dt`

---

## 2. Gyroscope
The **gyroscope** measures **angular velocity** (degrees per second, °/s), around the X (roll), Y (pitch) and Z (yaw) global axes.

They are computed by differentiating once the trajectory orientations:

`ω[i] = (ω[i] - ω[i - 1]) / dt`



## 3. Magnetometer
The **magnetometer** measures the Earth's magnetic field intensity in µT along the X, Y and Z local axes.

They are computed by applying a **rotation matrix** based on the current trajectory orientation, to the default Earth's magnetic field vector.

For example, a motion around the Z-axis (yaw), applies the following transformation:

`B[i] = R[i] x B_earth`

Where: `R[i] = Rz[i] x Ry[i] x Rx[i]` 

And `B_earth ≈ 1e-3 * [19.55, 335.9, 45.00] µT` at Greenwich according to the [NOAA](https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml).

Also: **Rx =**

| 1 | 0         | 0          |
|---|-----------|------------|
| 0 | cos(roll) | -sin(roll) |
| 0 | sin(roll) | cos(roll)  |

And: **Ry =**

| cos(pitch)  | 0 | sin(pitch) |
|-------------|---|------------|
| 0           | 1 | 0          |
| -sin(pitch) | 0 | cos(pitch) |

And: **Rz =**

| cos(yaw) | -sin(yaw) |  0  |
|----------|-----------|-----|
| sin(yaw) | cos(yaw)  |  0  |
| 0        | 0         |  1  |

---

## 4. Sensor Noise and Bias Drift
To increase realism, **random noise and slow sensor drift** are added:
- **Accelerometer Noise**: Gaussian noise with **σ = 0.05 m/s²**.
- **Gyroscope Noise**: Gaussian noise with **σ = 0.1°/s**.
- **Gyroscope Bias Drift**: Slow drift using **σ = 0.005°/s per step**.
- **Magnetometer Noise**: Minimal fluctuations (σ = 0.3 µT) to avoid disrupting rotation matrices.
- **Temperature Noise**: Small variations (σ = 0.1°C) to simulate environmental effects.

---

## 5. Simulated Serial Output
The simulated IMU sends **JSON-formatted sensor data** at **10 Hz** to a virtual serial port:

```json
{
  'timestamp': 3100.0, 
  'temperature': 25.15, 
  'accelerometer': {'x': -0.0764, 'y': 0.3874, 'z': 9.81}, 
  'gyroscope': {'x': -4.3826, 'y': -0.3722, 'z': 3.2556}, 
  'magnetometer': {'x': 27.3186, 'y': -16.7745, 'z': 45.7964}, 
  'quaternions': {'x': 0.0189, 'y': 0.0011, 'z': -0.0461, 'w': 0.9988}, 
  'euler': {'roll': -5.2803, 'pitch': 0.2284, 'yaw': 2.161},
}
```
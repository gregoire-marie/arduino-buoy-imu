# Inertial Measurement Unit (IMU) - Simulator

## Functioning
We differentiate the equations of motion for real-time IMU simulation: **accelerometer**, **gyroscope**, **magnetometer**, and **thermometer**.

### IMU Motion Cycle

| Phase | Duration (s) | Affected Sensor |
|--------|--------------|----------------|
| Forward / Backward | 2 each | Accelerometer |
| Left / Right | 2 each | Accelerometer |
| Up / Down | 2 each | Accelerometer |
| Roll Left / Right | 2 each | Gyroscope, Magnetometer |
| Pitch Up / Down | 2 each | Gyroscope, Magnetometer |
| Yaw Left / Right | 2 each | Gyroscope, Magnetometer |

**Sensor noise and bias drift** is added to resemble actual IMU behavior.

---

## 1. Kinematic Equations for Acceleration

From **kinematics**:

**d = (1/2) * a * t²**

Where:
- **d** = displacement in meters
- **a** = acceleration in m/s²
- **t** = time in seconds

Solving for acceleration:

**a = (2 * d) / t²**

Each phase moves the IMU by **2 meters in 2 seconds**, so:

**a = (2 * 2) / (2²) = 4 / 4 = 1.0 m/s²**

To ensure **smooth motion**, we multiply by a sinusoidal **easing function**:

**a(t) = A * sin(π * t)**

This ensures gradual acceleration and deceleration during each phase.

---

## 2. Rotational Motion Equations for Gyroscope
The **gyroscope** measures **angular velocity** (degrees per second, °/s), not acceleration.

For uniform angular motion:

**ω = θ / t**

Where:
- **θ** = total angle rotated (90° per phase)
- **t** = duration (2 sec per phase)

Thus, the expected angular velocity:

**ω = 90 / 2 = 45°/s**

To ensure smooth motion, we again apply an **easing function**:

**ω(t) = Ω * sin(π * t)**

where **Ω = 45°/s** is the max angular velocity per phase.

### Rotation Matrices for Magnetometer Data
The **magnetometer** measures Earth's magnetic field, which changes as the IMU rotates. We apply a **rotation matrix** based on the current phase’s motion.

For example, a **Yaw Left** motion applies the following transformation:

**R_yaw =**

| cos(θ)  | -sin(θ) |  0  |
|---------|---------|-----|
| sin(θ)  |  cos(θ) |  0  |
|   0     |    0    |  1  |

This is applied to the **Earth's magnetic field vector** to get the new magnetometer readings.

---

## 3. Sensor Noise and Bias Drift
To increase realism, **random noise and slow sensor drift** are added:
- **Accelerometer Noise**: Gaussian noise with **σ = 0.05 m/s²**.
- **Gyroscope Noise**: Gaussian noise with **σ = 0.1°/s**.
- **Gyroscope Bias Drift**: Slow drift using **σ = 0.005°/s per step**.
- **Magnetometer Noise**: Minimal fluctuations (σ = 0.3 µT) to avoid disrupting rotation matrices.
- **Temperature Noise**: Small variations (σ = 0.1°C) to simulate environmental effects.

---

## 4. Simulated Serial Output
The simulated IMU sends **JSON-formatted sensor data** at **10 Hz** to a virtual serial port:

```json
{
  "temperature": 24.97,
  "accelerometer": { "x": 0.5, "y": 0.0, "z": 9.81 },
  "gyroscope": { "x": 45.0, "y": 0.0, "z": 0.0 },
  "magnetometer": { "x": 24.5, "y": -31.2, "z": 38.9 },
  "phase": "Roll Left"
}
```


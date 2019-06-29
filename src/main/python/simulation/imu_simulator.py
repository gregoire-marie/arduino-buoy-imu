import datetime
import json
import time
import threading
import os
import pty  # Pseudo-terminal for proper serial simulation
import math
import numpy as np
from scipy.spatial.transform import Rotation as R  # For quaternion calculations

from config import SIMULATE_ARDUINO, PROJECT_DIR

# ------------------------------
# CONFIGURATION & SIMULATION MODE
# ------------------------------

# Create a pseudo-terminal (PTY) to simulate a serial port
TEMP_DIR = os.path.join(PROJECT_DIR, "simu_port")
os.makedirs(TEMP_DIR, exist_ok=True)
master, slave = pty.openpty()
SIMULATED_PORT = os.ttyname(slave)  # Get the name of the simulated serial device
TIME_STEP = 0.1  # Simulate a 10Hz sensor

# ------------------------------
# SENSOR SIMULATION PARAMETERS
# ------------------------------

# Physical Constants
G = 9.81  # Gravity (m/s²)
B_EARTH = np.array([25, -30, 40])  # Earth's magnetic field (µT)
TEMP_BASE = 25.0  # Base temperature (°C)
RAD_TO_DEG = 180 / math.pi  # Conversion factor from radians to degrees

# Noise Parameters
ACCEL_NOISE_STD = 0.05  # m/s² (low noise)
GYRO_NOISE_STD = 0.1  # degrees/s
GYRO_BIAS_STD = 0.005  # degrees/s per step (slow drift)
MAG_NOISE_STD = 0.3  # µT (minimal noise for stability)
TEMP_NOISE_STD = 0.1  # °C (environmental noise)

# Initialize gyroscope bias drift (persistent)
gyro_bias = np.array([0.0, 0.0, 0.0])

# Quaternion State
q = np.array([0.0, 0.0, 0.0, 1.0])  # Initial quaternion (no rotation)

# ------------------------------
# MOTION SIMULATION
# ------------------------------

# Motion sequence timing (seconds per phase)
MOTION_PHASES = [
    ("Forward", 2), ("Backward", 2),
    ("Left", 2), ("Right", 2),
    ("Up", 2), ("Down", 2),
    ("Roll Left", 2), ("Roll Right", 2),
    ("Pitch Up", 2), ("Pitch Down", 2),
    ("Yaw Left", 2), ("Yaw Right", 2)
]
TOTAL_MOTION_TIME = sum(t for _, t in MOTION_PHASES)

# Global time counter
start_time = time.time()


def get_motion_phase(t):
    """
    Determines the current motion phase based on elapsed time.

    :param t: Elapsed time since start
    :return: (phase, progress) where phase is the current motion and progress (0 to 1) is the phase completion percentage.
    """
    t = t % TOTAL_MOTION_TIME  # Ensure cyclic motion
    elapsed = 0
    for phase, duration in MOTION_PHASES:
        if elapsed <= t < elapsed + duration:
            return phase, (t - elapsed) / duration
        elapsed += duration
    return None, 0  # Should not occur


def easing_function(progress):
    """
    Generates a smooth acceleration profile using a sinusoidal function.

    :param progress: Normalized phase progress (0 to 1)
    :return: Adjusted progress value for smoother motion.
    """
    return math.sin(math.pi * progress)


def madgwick_update(gyro, accel, mag, dt):
    """
    Madgwick's IMU algorithm to compute quaternion orientation.
    :param gyro: 3D gyro vector (degrees/sec)
    :param accel: 3D accelerometer vector (m/s²)
    :param mag: 3D magnetometer vector (µT)
    :param dt: Time step (seconds)
    :return: Updated quaternion
    """
    global q
    gyro_rad = np.radians(gyro)  # Convert to radians/s

    # Convert to rotation object
    rotation_delta = R.from_rotvec(gyro_rad * dt)
    q = rotation_delta * R.from_quat(q)

    return q.as_quat()  # Return updated quaternion


def compute_rpy(q):
    """
    Converts a quaternion to Roll, Pitch, and Yaw (degrees).
    """
    r = R.from_quat(q)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)

    return roll, pitch, yaw


def sensors_from_phase(phase, progress):
    """ Returns accelerometer (m/s2), gyroscope (°/s) and magnetometer vectors (µT). """
    # Initialize default sensor readings
    accel = np.array([0.0, 0.0, G])  # Gravity vector m/s2
    gyro = np.array([0.0, 0.0, 0.0])  # Gyroscope readings °/s
    rotation_matrix = np.eye(3)  # No rotation by default

    # Smooth transition using easing function
    smooth_progress = easing_function(progress)

    # Apply motion effects
    if phase == "Forward":
        accel[0] = smooth_progress * (2 * 2 / (2 ** 2))
    elif phase == "Backward":
        accel[0] = -smooth_progress * (2 * 2 / (2 ** 2))
    elif phase == "Left":
        accel[1] = smooth_progress * (2 * 2 / (2 ** 2))
    elif phase == "Right":
        accel[1] = -smooth_progress * (2 * 2 / (2 ** 2))
    elif phase == "Up":
        accel[2] += smooth_progress * (2 * 2 / (2 ** 2))
    elif phase == "Down":
        accel[2] -= smooth_progress * (2 * 2 / (2 ** 2))

    # Rotational motion
    rot_angle = smooth_progress * math.pi / 2
    if phase == "Roll Left":
        gyro[0] = smooth_progress * 45  # °/s
        rotation_matrix = np.array(
            [[1, 0, 0],
             [0, math.cos(rot_angle), -math.sin(rot_angle)],
             [0, math.sin(rot_angle), math.cos(rot_angle)]])
    elif phase == "Roll Right":
        gyro[0] = -smooth_progress * 45
        rotation_matrix = np.array(
            [[1, 0, 0],
             [0, math.cos(rot_angle), -math.sin(rot_angle)],
             [0, math.sin(rot_angle), math.cos(rot_angle)]])

    elif phase == "Pitch Up":
        gyro[1] = smooth_progress * 45
        rotation_matrix = np.array(
            [[math.cos(rot_angle), 0, math.sin(rot_angle)],
             [0, 1, 0],
             [-math.sin(rot_angle), 0, math.cos(rot_angle)]])
    elif phase == "Pitch Down":
        gyro[1] = -smooth_progress * 45
        rotation_matrix = np.array(
            [[math.cos(rot_angle), 0, math.sin(rot_angle)],
             [0, 1, 0],
             [-math.sin(rot_angle), 0, math.cos(rot_angle)]])

    elif phase == "Yaw Left":
        gyro[2] = smooth_progress * 45
        rotation_matrix = np.array(
            [[math.cos(rot_angle), -math.sin(rot_angle), 0],
             [math.sin(rot_angle), math.cos(rot_angle), 0],
             [0, 0, 1]])
    elif phase == "Yaw Right":
        gyro[2] = -smooth_progress * 45
        rotation_matrix = np.array(
            [[math.cos(rot_angle), -math.sin(rot_angle), 0],
             [math.sin(rot_angle), math.cos(rot_angle), 0],
             [0, 0, 1]])

    # Compute rotated magnetic field
    mag = np.dot(rotation_matrix, B_EARTH)  # µT

    return accel, gyro, mag


def generate_fake_sensor_data():
    """
    Generates realistic IMU data simulating a sequential set of movements.

    - Simulated accelerometer (m/s²) with smooth motion and noise
    - Simulated gyroscope (°/s) with bias drift
    - Simulated magnetometer (µT) with small noise
    - Computed quaternions (1) from simulated sensors
    - Computed euler angles (°) from simulated sensors
    - Simulated temperature (°C) with slow variation

    :return: Dictionary containing simulated sensor values.
    """
    global gyro_bias, q
    t = time.time() - start_time
    phase, progress = get_motion_phase(t)

    # Get sensors
    accel, gyro, mag = sensors_from_phase(phase, progress)

    # Apply sensor noise
    accel += np.random.normal(0, ACCEL_NOISE_STD, 3)
    gyro += np.random.normal(0, GYRO_NOISE_STD, 3)

    # Simulate gyro bias drift
    gyro_bias += np.random.normal(0, GYRO_BIAS_STD, 3)
    gyro += gyro_bias

    # Apply small magnetometer noise
    mag += np.random.normal(0, MAG_NOISE_STD, 3)

    # Temperature variation
    temperature = TEMP_BASE + 0.5 * math.sin(t / 100) + np.random.normal(0, TEMP_NOISE_STD)

    # Compute quaternion update
    dt = TIME_STEP  # Simulation step size
    q = madgwick_update(gyro, accel, mag, dt)

    # Convert quaternion to RPY
    roll, pitch, yaw = compute_rpy(q)

    # Get current time
    timestamp = int(time.time() * 1000)

    # print(dict(zip(["x", "y", "z", "w"], q.round(4))))
    # print({"roll": round(roll, 4), "pitch": round(pitch, 4), "yaw": round(yaw, 4)})

    return {
        "timestamp": timestamp,
        "temperature": round(temperature, 2),
        "accelerometer": dict(zip(["x", "y", "z"], accel.round(4))),
        "gyroscope": dict(zip(["x", "y", "z"], gyro.round(4))),
        "magnetometer": dict(zip(["x", "y", "z"], mag.round(4))),
        "quaternions": dict(zip(["x", "y", "z", "w"], q.round(4))),
        "euler": {"roll": round(roll, 4), "pitch": round(pitch, 4), "yaw": round(yaw, 4)},
    }


def simulate_arduino_output():
    """Simulates Arduino continuously sending sensor data via a virtual serial port."""
    with open(master, "w") as ser:
        while SIMULATE_ARDUINO:
            fake_data = generate_fake_sensor_data()
            ser.write(json.dumps(fake_data) + "\n")
            ser.flush()
            time.sleep(TIME_STEP)  # Simulate sensor output frequency


def start_simulation():
    """Starts the simulated Arduino in a separate thread."""
    threading.Thread(target=simulate_arduino_output, daemon=True).start()

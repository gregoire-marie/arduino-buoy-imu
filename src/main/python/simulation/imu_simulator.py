import json
import time
import threading
import os
import pty  # Pseudo-terminal for proper serial simulation
import math
import numpy as np

from config import SIMULATE_ARDUINO, PROJECT_DIR
from src.main.python.simulation.filters import madgwick_update
from src.main.python.simulation.utils import rotation_matrix_from_euler, compute_rpy
from src.main.python.simulation.trajectory import Trajectory, OscillationTrajectory, \
    UniformTranslationRotationTrajectory, SpiralTrajectory

# ------------------------------
# CONFIGURATION & SIMULATION MODE
# ------------------------------

# Create a pseudo-terminal (PTY) to simulate a serial port
TEMP_DIR = os.path.join(PROJECT_DIR, "simu_port")
os.makedirs(TEMP_DIR, exist_ok=True)
master, slave = pty.openpty()
SIMULATED_PORT = os.ttyname(slave)  # Get the name of the simulated serial device
TIME_STEP = 0.1  # Simulate a 1/TIME_STEP Hz sensor. May be small for numerical stability
SERIAL_TIME_STEP = 0.1  # Approximate period of serial updates

# ------------------------------
# SENSOR SIMULATION PARAMETERS
# ------------------------------

# Duration
DURATION = 500  # s

# Physical Constants
G = 9.81  # Gravity (m/s²)
B_EARTH = np.array([25, -30, 40])  # Earth's magnetic field (µT)
TEMP_BASE = 25.0  # Base temperature (°C)
RAD_TO_DEG = 180 / math.pi  # Conversion factor from radians to degrees

# Noise Parameters
ACCEL_NOISE_STD = 0.05  # m/s² (low noise)
GYRO_NOISE_STD = 0.5  # degrees/s
GYRO_BIAS_STD = 0.001  # degrees/s per step (slow drift)
MAG_NOISE_STD = 0.3  # µT (minimal noise for stability)
TEMP_NOISE_STD = 0.1  # °C (environmental noise)

# Madgwick filter parameters
beta = 0.008660254037844387 # math.sqrt(3/4) * GYRO_NOISE_STD

# Initialize gyroscope bias drift (persistent)
gyro_bias = np.array([0.0, 0.0, 0.0])

# Quaternion State
q = np.array([0.0, 1e-9, 0.0, 1.0])  # Initial quaternion (no rotation)

# ------------------------------
# MOTION SIMULATION
# ------------------------------

# Define a trajectory
# trajectory = UniformTranslationRotationTrajectory(dt=TIME_STEP, duration=DURATION, velocity=(0, 0, 0), angular_velocity=(0, 0, 10))
trajectory = OscillationTrajectory(dt=TIME_STEP, duration=DURATION)
# trajectory = SpiralTrajectory(dt=TIME_STEP, duration=DURATION)


# Global time counter
start_time = time.time()

class IMUSimulator:
    """
    Simulates an Inertial Measurement Unit (IMU) with accelerometer, gyroscope, and magnetometer.
    """

    def __init__(self, trajectory: Trajectory):
        """
        Initializes the IMU simulator.

        Parameters:
        trajectory (Trajectory): The trajectory to be simulated.
        """
        self.trajectory = trajectory
        self.gravity = np.array([0, 0, -9.81])  # Gravity vector in world frame (m/s2)
        self.magnetic_field = B_EARTH  # Earth's magnetic field vector in world frame (µT)

    def simulate(self):
        """
        Simulates IMU sensor readings based on the provided trajectory.

        Returns:
        tuple: Position, orientation, accelerometer data, gyroscope data, magnetometer data.
        """
        position, orientation = self.trajectory.generate()
        num_steps = self.trajectory.num_steps
        time_data = self.trajectory.time_steps

        accelerometer_data = np.zeros((num_steps, 3))
        gyroscope_data = np.zeros((num_steps, 3))
        magnetometer_data = np.zeros((num_steps, 3))
        thermometer_data = np.zeros(num_steps)

        orientation_unwrapped = np.unwrap(np.radians(orientation), axis=0)  # Convert to radians and unwrap

        for i in range(1, num_steps - 1):
            roll, pitch, yaw = np.radians(orientation[i])
            R = rotation_matrix_from_euler(roll, pitch, yaw)

            # Compute acceleration
            velocity_prev = (position[i] - position[i - 1]) / self.trajectory.dt
            velocity_curr = (position[i + 1] - position[i]) / self.trajectory.dt
            acceleration = (velocity_curr - velocity_prev) / self.trajectory.dt
            accelerometer_data[i] = acceleration - self.gravity

            # Compute gyroscope values as angular velocity differences
            angular_velocity = np.degrees((orientation_unwrapped[i] - orientation_unwrapped[i - 1])) / self.trajectory.dt
            gyroscope_data[i] = angular_velocity

            # Rotate magnetic field into the IMU frame
            magnetometer_data[i] = np.dot(R, B_EARTH)  # µT

            # Compute temperature
            thermometer_data[i] = TEMP_BASE + 0.5 * math.sin(time_data[i] / 100) + np.random.normal(0, TEMP_NOISE_STD)

        return time_data, accelerometer_data, gyroscope_data, magnetometer_data, thermometer_data


def get_quat_euler(accel, gyro, mag):
    global gyro_bias, q

    # Apply sensor noise
    accel += np.random.normal(0, ACCEL_NOISE_STD, 3)
    gyro += np.random.normal(0, GYRO_NOISE_STD, 3)

    # Simulate gyro bias drift
    gyro_bias += np.random.normal(0, GYRO_BIAS_STD, 3)
    gyro += gyro_bias

    # Apply small magnetometer noise
    mag += np.random.normal(0, MAG_NOISE_STD, 3)

    # Compute quaternion update
    q = madgwick_update(q=q, accel=accel, gyro=gyro, mag=mag, dt=TIME_STEP, beta=beta)

    # Convert quaternion to RPY
    roll, pitch, yaw = compute_rpy(q)

    # For some reason, roll and yaw are inverted
    roll, yaw = yaw, roll

    return q, roll, pitch, yaw


def simulate_arduino_output():
    """Simulates Arduino continuously sending sensor data via a virtual serial port."""
    print("Generating Arduino data...")
    # Get a list of IMU values
    imu = IMUSimulator(trajectory)
    time_data, acc_data, gyro_data, mag_data, temp_data = imu.simulate()

    # Sample at serial frequency
    N = int(SERIAL_TIME_STEP / TIME_STEP)  # Subsampling index step
    if N > 0:
        time_data = time_data[::N]
        acc_data = acc_data[::N]
        gyro_data = gyro_data[::N]
        mag_data = mag_data[::N]
        temp_data = temp_data[::N]

    print("Sending via Serial port...")
    with open(master, "w") as ser:
        i = 0
        while SIMULATE_ARDUINO and i < len(acc_data):
            timestamp = time_data[i]
            accel, gyro, mag, temperature = acc_data[i], gyro_data[i], mag_data[i], temp_data[i]
            q, roll, pitch, yaw = get_quat_euler(accel=accel, gyro=gyro, mag=mag)
            fake_data = {
                "timestamp": timestamp * 1000,
                "temperature": round(temperature, 2),
                "accelerometer": dict(zip(["x", "y", "z"], accel.round(4))),
                "gyroscope": dict(zip(["x", "y", "z"], gyro.round(4))),
                "magnetometer": dict(zip(["x", "y", "z"], mag.round(4))),
                "quaternions": dict(zip(["x", "y", "z", "w"], q.round(4))),
                "euler": {"roll": round(roll, 4), "pitch": round(pitch, 4), "yaw": round(yaw, 4)},
            }
            ser.write(json.dumps(fake_data) + "\n")
            ser.flush()
            time.sleep(SERIAL_TIME_STEP)  # Simulate sensor output frequency
            i+=1


def start_simulation():
    """Starts the simulated Arduino in a separate thread."""
    threading.Thread(target=simulate_arduino_output, daemon=True).start()

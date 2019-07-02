import numpy as np
from ahrs.filters import Madgwick
from config import SIMULATE_ARDUINO


def madgwick_update(q, accel, gyro, mag, dt, beta):
    """
    Compute the orientation using the Madgwick filter from the `ahrs` library.

    Parameters:
        q (numpy array): Initial quaternion [q1, q2, q3, q4].
        accel (numpy array): Accelerometer readings [ax, ay, az]. in m/s2
        gyro (numpy array): Gyroscope readings [gx, gy, gz] in degree/s, converted to rad/s.
        mag (numpy array): Magnetometer readings [mx, my, mz] in µT.
        dt (float): Time step in seconds.
        beta (float): Algorithm gain parameter.

    Returns:
        numpy array: Updated quaternion [q1, q2, q3, q4].
    """
    # Initialize Madgwick filter instance
    madgwick = Madgwick(gain=beta, Dt=dt)

    def w_first(arr):
        """Swap the w component of a quaternion from last to first element of an array."""
        if arr.size > 1:  # Only swap if the array has at least two elements
            return np.array([arr[-1], arr[0], arr[1], arr[2]])
        return arr

    def w_last(arr):
        """Swap the w component of a quaternion from first to last element of an array."""
        if arr.size > 1:  # Only swap if the array has at least two elements
            return np.array([arr[1], arr[2], arr[3], arr[0]])
        return arr

    def swap_roll_yaw(arr):
        """Swap the roll and yaw components of a gyroscope between first and last elements of an array."""
        if arr.size > 1:  # Only swap if the array has at least two elements
            return np.array([arr[2], arr[1], arr[0]])
        return arr

    # Convert input to correct format
    q_w_first = w_first(q)
    gyro_rad = swap_roll_yaw(np.radians(gyro))  # Convert from degrees to radians
    mag_nano = mag * 1e3  # Convert from µT to nT
    q_new = madgwick.updateMARG(q_w_first, gyr=gyro_rad, acc=accel, mag=None if SIMULATE_ARDUINO else mag_nano)

    return w_last(q_new)

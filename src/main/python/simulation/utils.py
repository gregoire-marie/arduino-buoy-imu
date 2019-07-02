import numpy as np
from scipy.spatial.transform import Rotation as R


def rotation_matrix_from_euler(roll, pitch, yaw):
    """
    Computes the rotation matrix from Euler angles (ZYX convention).

    Parameters:
    roll (float): Rotation around X-axis in radians.
    pitch (float): Rotation around Y-axis in radians.
    yaw (float): Rotation around Z-axis in radians.

    Returns:
    np.ndarray: 3x3 rotation matrix.
    """
    Rz = np.array([
        [np.cos(yaw), -np.sin(yaw), 0],
        [np.sin(yaw), np.cos(yaw), 0],
        [0, 0, 1]
    ])

    Ry = np.array([
        [np.cos(pitch), 0, np.sin(pitch)],
        [0, 1, 0],
        [-np.sin(pitch), 0, np.cos(pitch)]
    ])

    Rx = np.array([
        [1, 0, 0],
        [0, np.cos(roll), -np.sin(roll)],
        [0, np.sin(roll), np.cos(roll)]
    ])

    return Rz @ Ry @ Rx


def compute_rpy(q):
    """
    Converts a quaternion (x, y, z, w) to euler angles (roll, pitch, yaw) in degrees.
    """
    r = R.from_quat(q)
    roll, pitch, yaw = r.as_euler('xyz', degrees=True)

    return roll, pitch, yaw

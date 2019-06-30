import matplotlib.pyplot as plt
import numpy as np
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

from src.main.python.serial.read_serial import MultiSubscriberSerialReader
from src.main.python.serial.sensor_data import SensorData

MAX_POINTS = 100

# 2D Raw Plot
start_time_2d_raw = None
queue_2d_raw = None
data2d_raw = SensorData(max_points=MAX_POINTS)

# 2D Angles Plot
start_time_2d_angles = None
queue_2d_angles = None
data2d_angles = SensorData(max_points=MAX_POINTS)

# 3D Angles Plot
queue_3d_angles = None
data3d_angles = SensorData(max_points=1)

# ------------------------------
# 2D RAW SENSORS DATA PLOT
# ------------------------------

def initialize_2d_raw_plot():
    """Sets up the raw data plots."""
    plt.style.use("classic")
    fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
    fig.suptitle("Real-Time Raw Sensor Data", fontsize=14, fontweight="bold")

    titles = ["Accelerometer (m/s²)", "Gyroscope (°/s)", "Magnetometer (µT)", "Temperature (°C)"]
    colors = [["r", "g", "b"], ["r", "g", "b"], ["r", "g", "b"], ["m"]]
    lines = []

    for i in range(4):
        axs[i].set_title(titles[i], fontsize=12, fontweight="bold")
        axs[i].grid(True, linestyle="--", linewidth=0.5, alpha=0.7)
        if i < 3:
            line_x, = axs[i].plot([], [], label="X", color=colors[i][0], linewidth=2)
            line_y, = axs[i].plot([], [], label="Y", color=colors[i][1], linewidth=2)
            line_z, = axs[i].plot([], [], label="Z", color=colors[i][2], linewidth=2)
            lines.append((line_x, line_y, line_z))
        else:
            line_temp, = axs[i].plot([], [], label="Temperature", color=colors[i][0], linewidth=2)
            lines.append(line_temp)
        axs[i].legend(loc="upper right", fontsize=10)

    axs[3].set_xlabel("Time (s)", fontsize=12, fontweight="bold")
    return fig, axs, lines


def update_2d_raw_plot(frame, serial_reader: MultiSubscriberSerialReader, lines, axs):
    """Updates plot in real-time."""
    global start_time_2d_raw
    global queue_2d_raw

    queue_2d_raw = serial_reader.subscribe() if queue_2d_raw is None else queue_2d_raw
    data = serial_reader.get_data(queue_2d_raw)
    if data:
        if start_time_2d_raw is None:
            start_time_2d_raw = data['timestamp']

        # Store sensor data
        data2d_raw.add_data(data)

        # Get elapsed time in seconds
        elapsed_time_data = data2d_raw.elapsed_time_sec

        # Update plot data
        for idx, dataset in enumerate([(data2d_raw.accel_x, data2d_raw.accel_y, data2d_raw.accel_z),
                                       (data2d_raw.gyro_x, data2d_raw.gyro_y, data2d_raw.gyro_z),
                                       (data2d_raw.mag_x, data2d_raw.mag_y, data2d_raw.mag_z)]
                                      ):
            lines[idx][0].set_data(elapsed_time_data, dataset[0])
            lines[idx][1].set_data(elapsed_time_data, dataset[1])
            lines[idx][2].set_data(elapsed_time_data, dataset[2])

        lines[3].set_data(elapsed_time_data, data2d_raw.temperature)
        axs[0].set_xlim(min(elapsed_time_data), max(elapsed_time_data) + 1)

        # Fixed y-axis ranges
        axs[0].set_ylim(-2, 12)
        axs[1].set_ylim(-200, 200)
        axs[2].set_ylim(-50, 50)
        axs[3].set_ylim(15, 35)

# ------------------------------
# 2D COMPUTED ANGLES PLOT
# ------------------------------

def initialize_2d_angles_plot():
    """Sets up the angles data plots."""
    plt.style.use("classic")
    fig, axs = plt.subplots(2, 1, figsize=(10, 10), sharex=True)
    fig.suptitle("Real-Time Computed Angles", fontsize=14, fontweight="bold")

    titles = ["Quaternions (1)", "Euler (°)"]
    colors = [["r", "g", "b", "m"], ["r", "g", "b"]]
    lines = []

    for i in range(2):
        axs[i].set_title(titles[i], fontsize=12, fontweight="bold")
        axs[i].grid(True, linestyle="--", linewidth=0.5, alpha=0.7)
        if i < 1:
            line_quatx, = axs[i].plot([], [], label="X", color=colors[i][0], linewidth=2)
            line_quaty, = axs[i].plot([], [], label="Y", color=colors[i][1], linewidth=2)
            line_quatz, = axs[i].plot([], [], label="Z", color=colors[i][2], linewidth=2)
            line_quatw, = axs[i].plot([], [], label="W", color=colors[i][3], linewidth=2)
            lines.append((line_quatx, line_quaty, line_quatz, line_quatw))
        else:
            line_yaw, = axs[i].plot([], [], label="Yaw", color=colors[i][0], linewidth=2)
            line_pitch, = axs[i].plot([], [], label="Pitch", color=colors[i][1], linewidth=2)
            line_roll, = axs[i].plot([], [], label="Roll", color=colors[i][2], linewidth=2)
            lines.append((line_yaw, line_pitch, line_roll))
        axs[i].legend(loc="upper right", fontsize=10)

    axs[1].set_xlabel("Time (s)", fontsize=12, fontweight="bold")
    return fig, axs, lines


def update_2d_angles_plot(frame, serial_reader: MultiSubscriberSerialReader, lines, axs):
    """Updates plot in real-time."""
    global start_time_2d_angles
    global queue_2d_angles

    queue_2d_angles = serial_reader.subscribe() if queue_2d_angles is None else queue_2d_angles
    data = serial_reader.get_data(queue_2d_angles)
    if data:
        if start_time_2d_angles is None:
            start_time_2d_angles = data['timestamp']

        # Store sensor data
        data2d_angles.add_data(data)

        # Get elapsed time in seconds
        elapsed_time_data = data2d_angles.elapsed_time_sec

        # Update plot data
        for idx, dataset in enumerate([(data2d_angles.quat_x, data2d_angles.quat_y, data2d_angles.quat_z, data2d_angles.quat_w),
                                       (data2d_angles.yaw, data2d_angles.pitch, data2d_angles.roll)]
                                      ):
            lines[idx][0].set_data(elapsed_time_data, dataset[0])
            lines[idx][1].set_data(elapsed_time_data, dataset[1])
            lines[idx][2].set_data(elapsed_time_data, dataset[2])
            lines[idx][3].set_data(elapsed_time_data, dataset[3]) if len(dataset) == 4 else None

        axs[0].set_xlim(min(elapsed_time_data), max(elapsed_time_data) + 1)

        # Fixed y-axis ranges
        axs[0].set_ylim(-1, 1)
        axs[1].set_ylim(-180, 180)

# ------------------------------
# 3D PLOT
# ------------------------------

def initialize_3d_plot():
    """Sets up the 3D plot with a cube in the middle."""
    plt.style.use("classic")
    fig = plt.figure(figsize=(8, 8))
    ax = fig.add_subplot(111, projection='3d')
    ax.set_title("Real-Time 3D Orientation", fontsize=14, fontweight="bold")

    # Define parallelepiped (Arduino-like) vertices
    length, width, height = 1.0, 0.5, 0.15  # Resembling an Arduino board
    arduino_vertices = np.array([[-length/2, -width/2, -height/2], [length/2, -width/2, -height/2],
                                  [length/2, width/2, -height/2], [-length/2, width/2, -height/2],
                                  [-length/2, -width/2, height/2], [length/2, -width/2, height/2],
                                  [length/2, width/2, height/2], [-length/2, width/2, height/2]])
    arduino_faces = [[arduino_vertices[j] for j in [0, 1, 2, 3]],
                     [arduino_vertices[j] for j in [4, 5, 6, 7]],
                     [arduino_vertices[j] for j in [0, 1, 5, 4]],
                     [arduino_vertices[j] for j in [2, 3, 7, 6]],
                     [arduino_vertices[j] for j in [0, 3, 7, 4]],
                     [arduino_vertices[j] for j in [1, 2, 6, 5]]]

    arduino = Poly3DCollection(arduino_faces, alpha=0.5, edgecolor='k')
    ax.add_collection3d(arduino)

    # Fixed reference axis in the top right
    ref_origin = np.array([0.7, 0.7, 0.7])
    ax.quiver(*ref_origin, 0.2, 0, 0, color='r', linewidth=2),  # X-axis (red)
    ax.quiver(*ref_origin, 0, 0.2, 0, color='g', linewidth=2),  # Y-axis (green)
    ax.quiver(*ref_origin, 0, 0, 0.2, color='b', linewidth=2)  # Z-axis (blue)

    # Add labels to fixed reference axes
    ax.text(ref_origin[0] + 0.25, ref_origin[1], ref_origin[2], 'X', color='r', fontsize=12, fontweight='bold')
    ax.text(ref_origin[0], ref_origin[1] + 0.25, ref_origin[2], 'Y', color='g', fontsize=12, fontweight='bold')
    ax.text(ref_origin[0], ref_origin[1], ref_origin[2] + 0.25, 'Z', color='b', fontsize=12, fontweight='bold')

    # Add parallelepiped local axis arrows
    local_origin = np.array([0, 0, 0])
    arrow_len = 0.1
    arduino_axes = [
        ax.quiver(*local_origin, arrow_len, 0, 0, color='r', linewidth=2),  # X-axis (red)
        ax.quiver(*local_origin, 0, arrow_len, 0, color='g', linewidth=2),  # Y-axis (green)
        ax.quiver(*local_origin, 0, 0, arrow_len, color='b', linewidth=2)  # Z-axis (blue)
    ]

    # # Remove 3D space background
    ax.set_xticklabels([])
    ax.set_yticklabels([])
    ax.set_zticklabels([])
    ax.grid(False)

    return fig, ax, arduino, arduino_vertices, arduino_axes


def update_3d_plot(frame, serial_reader: MultiSubscriberSerialReader, ax, cube, cube_vertices, cube_axes):
    """Updates plot in real-time."""
    global queue_3d_angles

    queue_3d_angles = serial_reader.subscribe() if queue_3d_angles is None else queue_3d_angles
    data = serial_reader.get_data(queue_3d_angles)

    if data is not None:
        data3d_angles.add_data(data)

        roll, pitch, yaw = np.radians(data3d_angles.roll[-1]), np.radians(data3d_angles.pitch[-1]), np.radians(
            data3d_angles.yaw[-1])

        # Rotation matrices
        Rx = np.array([[1, 0, 0], [0, np.cos(roll), -np.sin(roll)], [0, np.sin(roll), np.cos(roll)]])
        Ry = np.array([[np.cos(pitch), 0, np.sin(pitch)], [0, 1, 0], [-np.sin(pitch), 0, np.cos(pitch)]])
        Rz = np.array([[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]])

        # Apply rotation to the cube
        R = Rz @ Ry @ Rx
        rotated_vertices = np.dot(cube_vertices, R.T)

        # Update cube faces
        cube.set_verts([[rotated_vertices[j] for j in [0, 1, 2, 3]],
                        [rotated_vertices[j] for j in [4, 5, 6, 7]],
                        [rotated_vertices[j] for j in [0, 1, 5, 4]],
                        [rotated_vertices[j] for j in [2, 3, 7, 6]],
                        [rotated_vertices[j] for j in [0, 3, 7, 4]],
                        [rotated_vertices[j] for j in [1, 2, 6, 5]]])

        # Modify rotation for arrows (invert X-axis rotation)
        R_arrows = R.T

        # Update cube local axis arrows
        new_axes = 0.4 * np.dot(np.eye(3), R_arrows.T)  # Transform identity axes
        for i, (arrow, new_axis) in enumerate(zip(cube_axes, new_axes.T)):
            arrow.set_segments([[[0, 0, 0], new_axis.tolist()]])

        ax.set_xlim([-1, 1])
        ax.set_ylim([-1, 1])
        ax.set_zlim([-1, 1])
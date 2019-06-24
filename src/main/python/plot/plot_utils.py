import matplotlib.pyplot as plt

from src.main.python.serial.read_serial import MultiSubscriberSerialReader

MAX_POINTS = 100

START_TIME = None
queue_2d = None

def initialize_2d_plot():
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


def update_2d_plot(frame, serial_reader: MultiSubscriberSerialReader, time_data, accel_x, accel_y, accel_z,
                   gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature, lines, axs):
    """Updates plots and logs data to CSV in real-time."""
    global START_TIME
    global queue_2d

    queue_2d = serial_reader.subscribe() if queue_2d is None else queue_2d
    data = serial_reader.get_data(queue_2d)
    if data:
        timestamp = data['timestamp']
        time_data.append(timestamp)

        if START_TIME is None:
            START_TIME = timestamp

        # Store sensor data
        accel_x.append(data["accelerometer"]["x"])
        accel_y.append(data["accelerometer"]["y"])
        accel_z.append(data["accelerometer"]["z"])
        gyro_x.append(data["gyroscope"]["x"])
        gyro_y.append(data["gyroscope"]["y"])
        gyro_z.append(data["gyroscope"]["z"])
        mag_x.append(data["magnetometer"]["x"])
        mag_y.append(data["magnetometer"]["y"])
        mag_z.append(data["magnetometer"]["z"])
        temperature.append(data["temperature"])

        # Keep only the latest MAX_POINTS samples
        if len(time_data) > MAX_POINTS:
            time_data.pop(0)
            accel_x.pop(0); accel_y.pop(0); accel_z.pop(0)
            gyro_x.pop(0); gyro_y.pop(0); gyro_z.pop(0)
            mag_x.pop(0); mag_y.pop(0); mag_z.pop(0)
            temperature.pop(0)

        elapsed_time_data = [(time_data[i] - START_TIME) / 1e3 for i in range(len(time_data))]

        # Update plot data
        for idx, dataset in enumerate([(accel_x, accel_y, accel_z), (gyro_x, gyro_y, gyro_z), (mag_x, mag_y, mag_z)]):
            lines[idx][0].set_data(elapsed_time_data, dataset[0])
            lines[idx][1].set_data(elapsed_time_data, dataset[1])
            lines[idx][2].set_data(elapsed_time_data, dataset[2])

        lines[3].set_data(elapsed_time_data, temperature)
        axs[0].set_xlim(min(elapsed_time_data), max(elapsed_time_data) + 1)

        # Fixed y-axis ranges
        axs[0].set_ylim(-2, 12)
        axs[1].set_ylim(-200, 200)
        axs[2].set_ylim(-50, 50)
        axs[3].set_ylim(15, 35)

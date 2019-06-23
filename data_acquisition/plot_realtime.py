import matplotlib
matplotlib.use("TkAgg")  # Set a GUI-compatible backend

import matplotlib.pyplot as plt
import matplotlib.animation as animation
import csv
import os
from datetime import datetime
from read_serial import SerialReader
from simulation.imu_simulator import SIMULATED_ARDUINO, start_simulation, SIMULATED_PORT

# Set the serial port based on simulation mode
SERIAL_PORT = SIMULATED_PORT if SIMULATED_ARDUINO else "/dev/ttyUSB0"

# Start simulation if enabled
if SIMULATED_ARDUINO:
    start_simulation()

# Initialize Serial Reader
serial_reader = SerialReader(port=SERIAL_PORT)
serial_reader.connect()

# Data storage for plotting
time_data = []
accel_x, accel_y, accel_z = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []
mag_x, mag_y, mag_z = [], [], []
temperature = []

# Max points on the graph before shifting
MAX_POINTS = 100

# Create figure and subplots with a clean style
plt.style.use("classic")
fig, axs = plt.subplots(4, 1, figsize=(10, 10), sharex=True)
fig.suptitle("Real-Time Sensor Data", fontsize=14, fontweight="bold")

# Set labels and styles
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

# CSV File Setup
results_folder = f'../results/data_acquisition'
csv_filename = f"{results_folder}/sensor_data.csv"
if not os.path.isdir(csv_filename):
    os.makedirs(results_folder, exist_ok=True)

with open(csv_filename, mode="w", newline="") as file:
    writer = csv.writer(file)

    # Write headers only if file is new
    writer.writerow([
        "Timestamp",
        "Accel_X", "Accel_Y", "Accel_Z",
        "Gyro_X", "Gyro_Y", "Gyro_Z",
        "Mag_X", "Mag_Y", "Mag_Z",
        "Temperature"
    ])

def update_plot(frame):
    """Updates the plots in real time and writes to CSV."""
    global time_data, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature

    # Read data from Serial
    data = serial_reader.get_data()
    print(data)
    if data:
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        # Update time axis
        current_timestep = time_data[-1] + 1 if len(time_data) > 0 else 0
        time_data.append(current_timestep)

        # Store accelerometer data
        accel_x.append(data["accelerometer"]["x"])
        accel_y.append(data["accelerometer"]["y"])
        accel_z.append(data["accelerometer"]["z"])

        # Store gyroscope data
        gyro_x.append(data["gyroscope"]["x"])
        gyro_y.append(data["gyroscope"]["y"])
        gyro_z.append(data["gyroscope"]["z"])

        # Store magnetometer data
        mag_x.append(data["magnetometer"]["x"])
        mag_y.append(data["magnetometer"]["y"])
        mag_z.append(data["magnetometer"]["z"])

        # Store temperature data
        temperature.append(data["temperature"])

        # Keep only the latest MAX_POINTS samples
        if len(time_data) > MAX_POINTS:
            time_data.pop(0)
            accel_x.pop(0); accel_y.pop(0); accel_z.pop(0)
            gyro_x.pop(0); gyro_y.pop(0); gyro_z.pop(0)
            mag_x.pop(0); mag_y.pop(0); mag_z.pop(0)
            temperature.pop(0)

        # Update plots with new data
        lines[0][0].set_data(time_data, accel_x)
        lines[0][1].set_data(time_data, accel_y)
        lines[0][2].set_data(time_data, accel_z)

        lines[1][0].set_data(time_data, gyro_x)
        lines[1][1].set_data(time_data, gyro_y)
        lines[1][2].set_data(time_data, gyro_z)

        lines[2][0].set_data(time_data, mag_x)
        lines[2][1].set_data(time_data, mag_y)
        lines[2][2].set_data(time_data, mag_z)

        lines[3].set_data(time_data, temperature)

        # Adjust x-axis limits dynamically
        axs[0].set_xlim(min(time_data), max(time_data) + 1)

        # Auto scale y-limits
        axs[0].set_ylim(-2, 12)  # Fixed accelerometer range (example)
        axs[1].set_ylim(-200, 200)  # Fixed gyroscope range
        axs[2].set_ylim(-50, 50)  # Fixed magnetometer range
        axs[3].set_ylim(15, 35)  # Fixed temperature range

        # for i in range(4):
        #     axs[i].set_ylim(min(time_data), max(time_data) + 1)

        # Append data to CSV file
        with open(csv_filename, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                timestamp,
                accel_x[-1], accel_y[-1], accel_z[-1],
                gyro_x[-1], gyro_y[-1], gyro_z[-1],
                mag_x[-1], mag_y[-1], mag_z[-1],
                temperature[-1]
            ])

if __name__ == "__main__":
    # Set up animation
    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

    # Show plot
    plt.show()

    # Close Serial connection on exit
    serial_reader.close()
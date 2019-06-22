import matplotlib
matplotlib.use("TkAgg")  # Set a GUI-compatible backend

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from read_serial import SerialReader

# Adjust the serial port as necessary
SERIAL_PORT = "/dev/ttyUSB0"  # Linux/macOS: "/dev/ttyUSB0", Windows: "COM3"

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

# Create figure and subplots
fig, axs = plt.subplots(4, 1, figsize=(8, 10), sharex=True)

# Titles and labels
axs[0].set_title("Accelerometer (m/s²)")
axs[1].set_title("Gyroscope (°/s)")
axs[2].set_title("Magnetometer (µT)")
axs[3].set_title("Temperature (°C)")
axs[3].set_xlabel("Time (s)")

def update_plot(frame):
    """Updates the plots in real time."""
    global time_data, accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature

    # Read data from Serial
    data = serial_reader.get_data()
    if data:
        # Update time axis
        time_data.append(len(time_data))

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

        # Update plots
        axs[0].cla(); axs[0].set_title("Accelerometer (m/s²)")
        axs[0].plot(time_data, accel_x, label="X", color='r')
        axs[0].plot(time_data, accel_y, label="Y", color='g')
        axs[0].plot(time_data, accel_z, label="Z", color='b')
        axs[0].legend()

        axs[1].cla(); axs[1].set_title("Gyroscope (°/s)")
        axs[1].plot(time_data, gyro_x, label="X", color='r')
        axs[1].plot(time_data, gyro_y, label="Y", color='g')
        axs[1].plot(time_data, gyro_z, label="Z", color='b')
        axs[1].legend()

        axs[2].cla(); axs[2].set_title("Magnetometer (µT)")
        axs[2].plot(time_data, mag_x, label="X", color='r')
        axs[2].plot(time_data, mag_y, label="Y", color='g')
        axs[2].plot(time_data, mag_z, label="Z", color='b')
        axs[2].legend()

        axs[3].cla(); axs[3].set_title("Temperature (°C)")
        axs[3].plot(time_data, temperature, label="Temperature", color='m')
        axs[3].legend()


if __name__ == "__main__":
    # Set up animation
    ani = animation.FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

    # Show plot
    plt.show()

    # Close Serial connection on exit
    serial_reader.close()

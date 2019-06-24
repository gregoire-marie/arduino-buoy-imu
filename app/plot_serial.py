import matplotlib

from src.main.python.plot.plot_utils import (
    initialize_raw_plot, setup_csv, update_raw_plot
)
from src.main.python.serial.read_serial import ProcessedSerialReader

matplotlib.use("TkAgg")

import matplotlib.pyplot as plt
import matplotlib.animation as animation
from src.main.python.simulation.imu_simulator import start_simulation, SIMULATED_PORT
from config import SIMULATE_ARDUINO, ARDUINO_PORT, RESULTS_FOLDER

# Set the serial port
SERIAL_PORT = SIMULATED_PORT if SIMULATE_ARDUINO else ARDUINO_PORT

# Start simulation if enabled
if SIMULATE_ARDUINO:
    start_simulation()

# Initialize Serial Reader
serial_reader = ProcessedSerialReader(port=SERIAL_PORT, baud_rate=115200, timeout=1)
serial_reader.connect()

# Initialize data structures
time_data = []
accel_x, accel_y, accel_z = [], [], []
gyro_x, gyro_y, gyro_z = [], [], []
mag_x, mag_y, mag_z = [], [], []
temperature = []

# Setup raw data figure and CSV file
fig, axs, lines = initialize_raw_plot()
results_folder = f"{RESULTS_FOLDER}/serial"
csv_filename = setup_csv(results_folder)

# Start animations
raw_ani = animation.FuncAnimation(fig, update_raw_plot, interval=100, cache_frame_data=False,
                                  fargs=(serial_reader, time_data, accel_x, accel_y, accel_z,
                                        gyro_x, gyro_y, gyro_z, mag_x, mag_y, mag_z, temperature,
                                        lines, axs, csv_filename))

# Show plots
plt.show()

# Close Serial connection on exit
serial_reader.close()

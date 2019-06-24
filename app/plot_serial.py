import threading

import matplotlib

from src.main.python.plot.plot_utils import (
    initialize_2d_plot, update_2d_plot
)
from src.main.python.plot.save_utils import setup_csv, update_csv
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

# Setup raw data figure
fig_2d, axs_2d, lines_2d = initialize_2d_plot()

# Setup CSV file
csv_path = f"{RESULTS_FOLDER}/serial/sensor_data.csv"
csv_bool = setup_csv(csv_path, overwrite=True)

# Start CSV updater
if csv_bool:
    csv_updater = threading.Thread(target=update_csv, args=(serial_reader, csv_path), daemon=True).start()

# Start animations
ani_2d = animation.FuncAnimation(fig_2d, update_2d_plot, interval=1, cache_frame_data=False,
                                 fargs=(serial_reader, lines_2d, axs_2d))

# Show plots
plt.show()

# Close Serial connection on exit
serial_reader.close()

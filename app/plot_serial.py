import threading

import matplotlib

from src.main.python.plot.plot_utils import (
    initialize_2d_raw_plot, update_2d_raw_plot, initialize_2d_trajectory_plot, update_2d_trajectory_plot, initialize_3d_plot,
    update_3d_plot, initialize_2d_true_trajectory_plot, update_2d_true_trajectory_plot
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
fig_2d_raw, axs_2d_raw, lines_2d_raw = initialize_2d_raw_plot()
fig_2d_trajectory, axs_2d_trajectory, lines_2d_trajectory = initialize_2d_trajectory_plot()
# fig_2d_true_trajectory, axs_2d_true_trajectory, lines_2d_true_trajectory = initialize_2d_true_trajectory_plot()
fig_3d, axs_3d, cube, cube_vertices, cube_axes, timestamp_text = initialize_3d_plot()

# Setup CSV file
csv_path = f"{RESULTS_FOLDER}/serial/sensor_data.csv"
csv_bool = setup_csv(csv_path, overwrite=True)

# Start CSV updater
if csv_bool:
    csv_updater = threading.Thread(target=update_csv, args=(serial_reader, csv_path), daemon=True).start()

# Start animations
ani_2d_raw = animation.FuncAnimation(fig_2d_raw, update_2d_raw_plot, interval=10, cache_frame_data=False,
                                     fargs=(serial_reader, lines_2d_raw, axs_2d_raw))

ani_2d_trajectory = animation.FuncAnimation(fig_2d_trajectory, update_2d_trajectory_plot, interval=10, cache_frame_data=False,
                                            fargs=(serial_reader, lines_2d_trajectory, axs_2d_trajectory))

# ani_2d_true_trajectory = animation.FuncAnimation(fig_2d_true_trajectory, update_2d_true_trajectory_plot, interval=10, cache_frame_data=False,
#                                                  fargs=(serial_reader, lines_2d_true_trajectory, axs_2d_true_trajectory))

ani_3d = animation.FuncAnimation(fig_3d, update_3d_plot, interval=10, cache_frame_data=False,
                                 fargs=(serial_reader, axs_3d, cube, cube_vertices, cube_axes, timestamp_text))

# Show plots
plt.show()

# Close Serial connection on exit
serial_reader.close()

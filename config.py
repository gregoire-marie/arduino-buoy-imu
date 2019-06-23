import os

# Toggle to switch between real Arduino and simulated IMU
SIMULATE_ARDUINO = True

# Arduino Serial Port
ARDUINO_PORT = "/dev/ttyUSB0"  # Linux/macOS: "/dev/ttyUSB0", Windows: "COM3"

# Data Storage Paths
RESULTS_FOLDER = os.path.join("..", "results", "serial")
CSV_FILE = os.path.join(RESULTS_FOLDER, "sensor_data.csv")

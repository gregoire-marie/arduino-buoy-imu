import os
PROJECT_DIR = os.path.dirname(__file__)

# Toggle to switch between real Arduino and simulated IMU
SIMULATE_ARDUINO = True

# Arduino Serial Port
ARDUINO_PORT = "/dev/ttyUSB0"  # Linux/macOS: "/dev/ttyUSB0", Windows: "COM3"

# Results Storage Folder
RESULTS_FOLDER = os.path.join(PROJECT_DIR, "results")

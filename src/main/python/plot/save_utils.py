import csv
import os

from src.main.python.serial.read_serial import MultiSubscriberSerialReader


def setup_csv(csv_path: str, overwrite: bool):
    """Ensures the results folder exists and initializes the CSV file.
    Args:
        csv_path (str): Path to the CSV file that will be created.
        overwrite (bool): Whether to overwrite the existing CSV file if it exists.
    """
    csv_folder = os.path.dirname(csv_path)
    os.makedirs(csv_folder, exist_ok=True)

    if overwrite and os.path.exists(csv_path):
        print("CSV file already exists. Overwriting")
        # Remove existing file
        os.remove(csv_path)

    if not os.path.exists(csv_path):
        with open(csv_path, mode="w", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                "Timestamp",
                "Accel_X", "Accel_Y", "Accel_Z",
                "Gyro_X", "Gyro_Y", "Gyro_Z",
                "Mag_X", "Mag_Y", "Mag_Z",
                "Temperature"
            ])
    else:
        print(f'CSV file already exists: {csv_path}')


def update_csv(serial_reader: MultiSubscriberSerialReader, csv_filename):
    queue_csv = serial_reader.subscribe()

    while serial_reader.running:
        data = serial_reader.get_data(queue_csv)

        # Append to CSV
        with open(csv_filename, mode="a", newline="") as file:
            writer = csv.writer(file)
            writer.writerow([
                data["timestamp"],
                data["accelerometer"]["x"], data["accelerometer"]["y"], data["accelerometer"]["z"],
                data["gyroscope"]["x"], data["gyroscope"]["y"], data["gyroscope"]["z"],
                data["magnetometer"]["x"], data["magnetometer"]["y"], data["magnetometer"]["z"],
                data["temperature"]
            ])

import serial
import json
import threading
import queue

class SerialReader:
    """
    Handles serial communication with the Arduino and retrieves JSON data.
    """
    def __init__(self, port: str, baud_rate: int = 115200, timeout: float = 1.0):
        """
        Initialize the serial connection.

        :param port: The serial port to connect to (e.g., "/dev/ttyUSB0" or "COM3").
        :param baud_rate: The baud rate for serial communication.
        :param timeout: Read timeout for the serial port.
        """
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial_conn = None
        self.data_queue = queue.Queue()
        self.running = False

    def connect(self):
        """Establishes the serial connection."""
        try:
            self.serial_conn = serial.Serial(self.port, self.baud_rate, timeout=self.timeout)
            self.running = True
            threading.Thread(target=self._read_serial, daemon=True).start()
        except serial.SerialException as e:
            print(f"Error opening serial port {self.port}: {e}")
            self.running = False

    def _read_serial(self):
        """Continuously reads serial data and places it into a queue."""
        while self.running:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    data = json.loads(line)
                    self.data_queue.put(data)
            except (json.JSONDecodeError, UnicodeDecodeError):
                print("Error decoding JSON data from serial.")

    def get_data(self):
        """
        Retrieves the latest available data from the queue.

        :return: A dictionary with the latest sensor readings or None if no data.
        """
        try:
            return self.data_queue.get_nowait()
        except queue.Empty:
            return None

    def close(self):
        """Closes the serial connection."""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()


class ProcessedSerialReader(SerialReader):
    def __init__(self, *args, **kwargs):
        """
        Initializes ProcessedSerialReader, which extends SerialReader to include additional
        data processing.
        """
        SerialReader.__init__(self, *args, **kwargs)

    def get_data(self):
        """
        Overrides the get_data() method to fetch raw sensor data and apply additional processing.

        :return: Processed sensor data dictionary.
        """
        try:
            data = self.data_queue.get_nowait()
        except queue.Empty:
            return None

        # Ensure data is not empty before processing
        if not data:
            return None

        # Apply Filtering (e.g., Kalman, Complementary)
        data["filtered_accel"] = self.apply_kalman_filter(data["accelerometer"])
        data["filtered_gyro"] = self.apply_kalman_filter(data["gyroscope"])
        data["filtered_mag"] = self.apply_kalman_filter(data["magnetometer"])

        return data

    def apply_kalman_filter(self, sensor_data):
        """
        Applies a simple Kalman filter (or another filter) to smooth sensor readings.

        :param sensor_data: Raw sensor dictionary {x, y, z}.
        :return: Filtered sensor dictionary {x, y, z}.
        """
        # TODO: Implement actual Kalman filter
        # For now, return raw data (placeholder)
        return sensor_data
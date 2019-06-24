import serial
import json
import threading
import queue

from src.main.python.serial.sensor_data import SensorData


class SerialReader:
    """
    Handles serial communication with the Arduino and retrieves JSON data.
    """
    def __init__(self, port: str, baud_rate: int, timeout: float, *args, **kwargs):
        """
        Initialize the serial connection.

        :param port: The serial port to connect to (e.g., "/dev/ttyUSB0" or "COM3").
        :param baud_rate: The baud rate for serial communication (e.g., 115200).
        :param timeout: Read timeout for the serial port (e.g., 1.0).
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
            self.serial_conn = serial.Serial(port=self.port, baudrate=self.baud_rate, timeout=self.timeout)
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
        Waits and retrieves the latest available data from the queue.

        :return: A dictionary with the latest sensor readings.
        """
        return self.data_queue.get()

    def close(self):
        """Closes the serial connection."""
        self.running = False
        if self.serial_conn:
            self.serial_conn.close()


class MultiSubscriberSerialReader(SerialReader):
    """
    Extends SerialReader to support multiple subscribers using independent queues.
    """
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.subscribers = []  # List of queues for subscribers

    def _read_serial(self):
        """Continuously reads serial data and distributes it to all subscriber queues."""
        while self.running:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    data = json.loads(line)
                    # Distribute data to all subscriber queues
                    for subscriber_queue in self.subscribers:
                        subscriber_queue.put(data)
            except (json.JSONDecodeError, UnicodeDecodeError):
                print("Error decoding JSON data from serial.")

    def subscribe(self):
        """Allows multiple processes to subscribe and receive independent copies of data."""
        new_queue = queue.Queue()
        self.subscribers.append(new_queue)
        return new_queue

    def get_data(self, subscriber_queue: queue.Queue = None):
        """
        Waits and retrieves the latest available data from the given subscriber queue.

        :param subscriber_queue: The queue associated with the subscriber.
        :return: A dictionary with the latest sensor readings.
        """
        if subscriber_queue is None:
            return super().get_data()
        return subscriber_queue.get()

    def close(self):
        """Closes the serial connection and stops reading data."""
        super().close()
        self.subscribers.clear()  # Clear all subscriber queues


class ProcessedSerialReader(MultiSubscriberSerialReader):
    def __init__(self, *args, **kwargs):
        """
        Extends MultiSubscriberSerialReader to include additional data processing.
        """
        super().__init__(*args, **kwargs)
        self.historical_data = SensorData()
        self.processed_historical_data = SensorData()

    def _read_serial(self):
        """Continuously reads serial data and distributes it to all subscriber queues."""
        while self.running:
            try:
                line = self.serial_conn.readline().decode('utf-8').strip()
                if line:
                    # Get new data
                    data = json.loads(line)
                    # Process new data in light of historical data
                    processed_data = self.process_data(data)
                    # Add to historical data
                    self.historical_data.add_data(data)
                    self.processed_historical_data.add_data(processed_data)
                    # Distribute data to all subscriber queues
                    for subscriber_queue in self.subscribers:
                        subscriber_queue.put(data)
            except (json.JSONDecodeError, UnicodeDecodeError):
                print("Error decoding JSON data from serial.")

    def process_data(self, sensor_data):
        """
        Applies a simple Kalman filter (or another filter) to smooth sensor readings.

        :param sensor_data: Raw sensor dictionary. Is such as: {'temperature': 25.1107, 'accelerometer': {'x': 0.4392, 'y': 0.0005, 'z': 9.8341}, 'gyroscope': {'x': 0.0032, 'y': 0.0487, 'z': 0.1067}, 'magnetometer': {'x': 25.3541, 'y': -29.5193, 'z': 39.9943}, 'quaternions': {'x': 123.8545, 'y': 32.1547, 'z': 14.4445, 'w': 0.0456}, 'euler': {'yaw': 0.8135, 'pitch': 12.8542, 'roll': 12.8042}}. Contains the same json type as the items in self.historical_data and self.processed_historical_data
        :return: Filtered sensor dictionary.
        """

        historical_data = self.historical_data
        processed_data = self.processed_historical_data
        current_data = sensor_data

        if historical_data:
            filtered_data = current_data  # TODO: Implement actual Kalman filter

        return sensor_data
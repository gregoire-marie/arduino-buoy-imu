class SensorData:
    def __init__(self, max_points=None):
        self.max_points = max_points
        self.time_data = []
        self.accel_x, self.accel_y, self.accel_z = [], [], []
        self.gyro_x, self.gyro_y, self.gyro_z = [], [], []
        self.mag_x, self.mag_y, self.mag_z = [], [], []
        self.quat_x, self.quat_y, self.quat_z, self.quat_w = [], [], [], []
        self.roll, self.pitch, self.yaw = [], [], []
        self.temperature = []

        self.true_x, self.true_y, self.true_z = [], [], []
        self.true_roll, self.true_pitch, self.true_yaw = [], [], []

        self.start_time = None
        self.elapsed_time_sec = []

    def add_data(self, data):
        """Add new sensor data while maintaining buffer size."""
        self.time_data.append(data["timestamp"])
        # Accelerometer
        self.accel_x.append(data["accelerometer"]["x"])
        self.accel_y.append(data["accelerometer"]["y"])
        self.accel_z.append(data["accelerometer"]["z"])
        # Gyroscope
        self.gyro_x.append(data["gyroscope"]["x"])
        self.gyro_y.append(data["gyroscope"]["y"])
        self.gyro_z.append(data["gyroscope"]["z"])
        # Magnetometer
        self.mag_x.append(data["magnetometer"]["x"])
        self.mag_y.append(data["magnetometer"]["y"])
        self.mag_z.append(data["magnetometer"]["z"])
        # Quaternions
        self.quat_x.append(data["quaternions"]["x"])
        self.quat_y.append(data["quaternions"]["y"])
        self.quat_z.append(data["quaternions"]["z"])
        self.quat_w.append(data["quaternions"]["w"])
        # Euler angles
        self.roll.append(data["euler"]["roll"])
        self.pitch.append(data["euler"]["pitch"])
        self.yaw.append(data["euler"]["yaw"])
        self.temperature.append(data["temperature"])

        # True position and orientation
        if "true_position" in list(data.keys()):
            self.true_x.append(data["true_position"]["x"])
            self.true_y.append(data["true_position"]["y"])
            self.true_z.append(data["true_position"]["z"])
        if "true_euler" in list(data.keys()):
            self.true_roll.append(data["true_euler"]["roll"])
            self.true_pitch.append(data["true_euler"]["pitch"])
            self.true_yaw.append(data["true_euler"]["yaw"])

        if self.start_time is None:
            self.start_time = data['timestamp']
        self.elapsed_time_sec.append((data['timestamp'] - self.start_time) / 1e3)

        # Keep buffer size constant
        if self.max_points is not None and len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.accel_x.pop(0); self.accel_y.pop(0); self.accel_z.pop(0)
            self.gyro_x.pop(0); self.gyro_y.pop(0); self.gyro_z.pop(0)
            self.mag_x.pop(0); self.mag_y.pop(0); self.mag_z.pop(0)
            self.quat_x.pop(0); self.quat_y.pop(0); self.quat_z.pop(0); self.quat_w.pop(0)
            self.roll.pop(0); self.pitch.pop(0); self.yaw.pop(0)
            self.temperature.pop(0)

            if len(self.true_x) > 0:
                self.true_x.pop(0); self.true_y.pop(0); self.true_z.pop(0);
            if len(self.true_roll) > 0:
                self.true_roll.pop(0); self.true_pitch.pop(0); self.true_yaw.pop(0)

            self.elapsed_time_sec.pop(0)

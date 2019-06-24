class SensorData:
    def __init__(self, max_points=None):
        self.max_points = max_points
        self.time_data = []
        self.accel_x, self.accel_y, self.accel_z = [], [], []
        self.gyro_x, self.gyro_y, self.gyro_z = [], [], []
        self.mag_x, self.mag_y, self.mag_z = [], [], []
        self.yaw, self.pitch, self.roll = [], [], []
        self.quat_x, self.quat_y, self.quat_z, self.quat_w = [], [], [], []
        self.temperature = []

    def add_data(self, data):
        """Add new sensor data while maintaining buffer size."""
        self.time_data.append(data["timestamp"])
        self.accel_x.append(data["accelerometer"]["x"])
        self.accel_y.append(data["accelerometer"]["y"])
        self.accel_z.append(data["accelerometer"]["z"])
        self.gyro_x.append(data["gyroscope"]["x"])
        self.gyro_y.append(data["gyroscope"]["y"])
        self.gyro_z.append(data["gyroscope"]["z"])
        self.mag_x.append(data["magnetometer"]["x"])
        self.mag_y.append(data["magnetometer"]["y"])
        self.mag_z.append(data["magnetometer"]["z"])
        # self.yaw.append(data["euler"]["yaw"])
        # self.pitch.append(data["euler"]["pitch"])
        # self.roll.append(data["euler"]["roll"])
        # self.quat_x.append(data["quaternions"]["x"])
        # self.quat_y.append(data["quaternions"]["y"])
        # self.quat_z.append(data["quaternions"]["z"])
        # self.quat_w.append(data["quaternions"]["w"])
        self.temperature.append(data["temperature"])

        # Keep buffer size constant
        if self.max_points is not None and len(self.time_data) > self.max_points:
            self.time_data.pop(0)
            self.accel_x.pop(0); self.accel_y.pop(0); self.accel_z.pop(0)
            self.gyro_x.pop(0); self.gyro_y.pop(0); self.gyro_z.pop(0)
            self.mag_x.pop(0); self.mag_y.pop(0); self.mag_z.pop(0)
            self.yaw.pop(0); self.pitch.pop(0); self.roll.pop(0)
            self.quat_x.pop(0); self.quat_y.pop(0); self.quat_z.pop(0); self.quat_w.pop(0)
            self.temperature.pop(0)

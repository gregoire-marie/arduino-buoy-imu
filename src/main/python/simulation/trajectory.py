import numpy as np


class Trajectory:
    """
    Base class for defining trajectories.
    """

    def __init__(self, dt=0.01, duration=10):
        """
        Initializes the trajectory parameters.

        Parameters:
        dt (float): Time step in seconds.
        duration (float): Total simulation duration in seconds.
        """
        self.dt = dt  # Time step
        self.duration = duration  # Total simulation time
        self.num_steps = int(duration / dt)
        self.time_steps = np.arange(0, self.duration, self.dt)

    def generate(self):
        """
        Generates the trajectory data (position and orientation over time).

        Returns:
        tuple: Position (Nx3 array) and orientation (Nx3 array of Euler angles).
        """
        raise NotImplementedError("Subclasses must implement the generate method.")


class UniformTranslationRotationTrajectory(Trajectory):
    """
    Defines a trajectory with constant linear and angular velocity.
    """

    def __init__(self, dt=0.01, duration=10, velocity=(0, 0, 0), angular_velocity=(0, 0, 0)):
        super().__init__(dt, duration)
        self.velocity = np.array(velocity)  # Constant linear velocity (m/s)
        self.angular_velocity = np.array(angular_velocity)  # Constant angular velocity (roll, pitch, yaw) (°/s)

    def generate(self):
        """
        Generates constant trajectory data.

        Returns:
        tuple: Position (Nx3 array) and orientation (Nx3 array of Euler angles).
        """
        position = np.zeros((self.num_steps, 3))
        orientation = np.zeros((self.num_steps, 3))  # Euler angles (roll, pitch, yaw)

        for i in range(1, self.num_steps):
            position[i] = position[i-1] + self.velocity * self.dt
            orientation[i] = orientation[i-1] + self.angular_velocity * self.dt

        # Wrap orientation angles to stay in the range [-180, 180]
        orientation = (orientation + 180) % 360 - 180

        return position, orientation


class OscillationTrajectory(Trajectory):
    """
    Defines a oscillatory trajectory.
    """

    def __init__(self, dt=0.01, duration=10):
        super().__init__(dt, duration)

    def generate(self):
        """
        Generates non-constant trajectory data (e.g., spiral motion).

        Returns:
        tuple: Position (Nx3 array) and orientation (Nx3 array of Euler angles).
        """
        position = np.zeros((self.num_steps, 3))
        orientation = np.zeros((self.num_steps, 3))  # Euler angles (roll, pitch, yaw)
        oscill_freq = 0.05
        omega = oscill_freq * 2 * np.pi

        for i in range(self.num_steps):
            t = self.time_steps[i]
            position[i] = np.array([0.1 * t, 0, 2.0 * np.sin(omega * t)])  # Example: Oscillatory altitude
            orientation[i] = np.array([0, 25.0 * np.sin(omega * t), 0])  # Example: Oscillatory pitch

        return position, orientation


class SpiralTrajectory(Trajectory):
    """
    Defines a trajectory with non-constant movement, such as oscillatory motion.
    """

    def __init__(self, dt=0.01, duration=10):
        super().__init__(dt, duration)

    def generate(self):
        """
        Generates non-constant trajectory data (e.g., spiral motion).

        Returns:
        tuple: Position (Nx3 array) and orientation (Nx3 array of Euler angles).
        """
        num_turns = 5  # Number of full turns
        radius = 100  # Radius of the spiral
        height = 100  # Total descent
        start_angle = - np.pi / 2
        theta = np.linspace(start_angle, start_angle + num_turns * 2 * np.pi, self.num_steps)  # Angle values

        position = np.zeros((self.num_steps, 3))
        orientation = np.zeros((self.num_steps, 3))  # Euler angles (roll, pitch, yaw)

        # Compute position
        x = radius * np.cos(theta)
        y = radius * np.sin(theta)
        z = - (height / (num_turns * 2 * np.pi)) * theta  # Linear descent

        # Start at the origin
        x -= x[0]
        y -= y[0]
        z -= z[0]

        # Compute orientation
        dx = np.gradient(x)
        dy = np.gradient(y)
        dz = np.gradient(z)

        roll = -np.cos(theta)  # Simulated banking
        pitch = np.arctan2(dz, np.sqrt(dx ** 2 + dy ** 2))  # Descent angle
        yaw = np.arctan2(dy, dx)  # Direction of motion

        for i in range(self.num_steps):
            position[i] = np.array([x[i], y[i], z[i]])  # Example: Spiral motion
            orientation[i] = np.degrees(np.array([roll[i], pitch[i], yaw[i]]))  # Example: Oscillatory rotation

        return position, orientation
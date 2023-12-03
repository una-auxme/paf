import numpy as np
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
# from std_msgs.msg import Float32

'''
This class implements a Kalman filter for a 3D object tracked in 3D space.
It implements the data of the IMU and the GPS Sensors.
The IMU Sensor provides the acceleration
and the GPS Sensor provides the position.
The IMU Data therefore needs to be transformed to Velocity data.

The Noise for the GPS Sensor is defined as:
    "noise_alt_stddev": 0.000005,
    "noise_lat_stddev": 0.000005,
    "noise_lon_stddev": 0.000005
The Noise for the IMU Sensor is defined as:
    "noise_accel_stddev_x": 0.000,
    "noise_accel_stddev_y": 0.000,
    "noise_accel_stddev_z": 0.000,

The state vector x is defined as:
    x = [x, y, z, vx, vy, vz]
    x: position
    v: velocity
The state transition matrix F is defined as:
    F = I
    I: Identity Matrix
The measurement matrix H is defined as:
    H =
The process covariance matrix Q is defined as:
    Q =
'''


class KalmanFilter(CompatibleNode):
    NOISE_ALT_STDDEV = 0.000005
    NOISE_LAT_STDDEV = 0.000005
    NOISE_LON_STDDEV = 0.000005

    def __init__(self):
        # Initialize the state vector x
        self.x = np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0]).T
        # Initialize Predected state vector x_hat
        self.x_hat = np.matrix([0, 0, 0, 0, 0, 0, 0, 0, 0]).T
        # Initialize the state transition matrix F
        self.F = np.matrix(np.identity(9))
        # Initialize the measurement matrix H
        # self.H =
        # Initialize the process covariance matrix Q
        self.Q = np.matrix(np.zeros((9, 9)))
        # Initialize the measurement covariance matrix R
        self.R = np.matrix(np.identity(9))
        # Initialize the covariance matrix P
        self.P = np.matrix(np.identity(9))
        # Initialize the predicted covariance matrix P_hat
        self.P_hat = np.matrix(np.identity(9))
        # Initialize the acceleration
        self.acceleration = np.matrix([0, 0, 0]).T
        # Initialize the position
        self.position = np.matrix([0, 0, 0]).T
        # Initialize the velocity
        self.velocity = np.matrix([0, 0, 0]).T
        # Initialize the time
        self.time = 0
        # Initialize the flag if the filter is initialized
        self.initialized = False

        # Subscriber
        # Initialize the subscriber for the IMU Data
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/Ideal_IMU",
            self.update_imu_data,
            qos_profile=1)
        # Initialize the subscriber for the GPS Data
        self.gps_subscriber = self.new_subscription(
            PoseStamped,
            "/carla/" + self.role_name + "/GPS",
            self.update_gps_data,
            qos_profile=1)

        # Publisher
        # Initialize the publisher for the kalman-position
        self.kalman_position_publisher = self.new_publisher(
            PoseStamped,
            "/carla/" + self.role_name + "/Kalman_Position",
            qos_profile=1)

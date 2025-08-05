#!/usr/bin/env python

"""
This node publishes the position and heading estimated by
a (linear) Kalman Filter on the topics:
  - kalman_pos
  - kalman_heading

The estimation is based on the data provided by the following sensors / topics:
  - IMU -> provides acceleration
  - unfiltered_pos (derived from the GPS sensor data) -> provides position
  - Speedometer -> provides the speed in the current direction

The state estimation is only done in a 2D plane.
To estimate a 3D position, the z-coordinate of the car is calculated
using a running average of the last GPS_RUNNING_AVG_ARGS measurements.
The z-coordinate is therefore not estimated by the Kalman Filter.

"""

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32, UInt32, String
from sensor_msgs.msg import NavSatFix, Imu
from carla_msgs.msg import CarlaSpeedometer
import math
from coordinate_transformation import CoordinateTransformer
from coordinate_transformation import quat_to_heading
from xml.etree import ElementTree as eTree
from paf_common.parameters import update_attributes

GPS_RUNNING_AVG_ARGS = 10

"""
DEFINITIONS:

Noise values are derived from:
https://github.com/carla-simulator/leaderboard/blob/leaderboard-2.0/leaderboard/autoagents/agent_wrapper.py

The Noise for the GPS Sensor is defined as:
    "noise_alt_stddev": 0.000005,
    "noise_lat_stddev": 0.000005,
    "noise_lon_stddev": 0.000005
The Noise for the IMU Sensor is defined as:
    "noise_accel_stddev_x": 0.001,
    "noise_accel_stddev_y": 0.001,
    "noise_accel_stddev_z": 0.015,

The state vector X is defined as:
            [initial_x],
            [initial_y],
            [v_x],
            [v_y],
            [yaw],
            [omega_z],
The state transition matrix F is defined as:
    A = np.array([[1, 0, self.dt, 0, 0, 0],
                    [0, 1, 0, self.dt, 0, 0],
                    [0, 0, 1, 0, 0, self.dt],
                    [0, 0, 0, 1, 0, 0],
                    [0, 0, 0, 0, 1, 0],
                    [0, 0, 0, 0, 0, 1]])
The measurement matrix H is defined as:
    self.H = np.array([[1, 0, 0, 0, 0, 0],    # x
                        [0, 1, 0, 0, 0, 0],   # y
                        [0, 0, 1, 0, 0, 0],   # v_x
                        [0, 0, 0, 1, 0, 0],   # v_y
                        [0, 0, 0, 0, 1, 0],   # yaw
                        [0, 0, 0, 0, 0, 1]])  # omega_z
The process covariance matrix Q is defined as:
    self.Q = np.diag([0.0001, 0.0001, 0.00001, 0.00001, 0.000001, 0.00001])
The measurement covariance matrix R is defined as:
    self.R = np.diag([0.0007, 0.0007, 0, 0, 0, 0])
"""


class KalmanFilter(Node):
    """
    This class implements a Kalman filter to estimate the
    position and heading of the car.
    """

    def __init__(self):
        """
        Constructor / Setup
        :return:
        """
        super().__init__("kalman_filter_node")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # basic info
        self.transformer = None  # for coordinate transformation
        self.control_loop_rate = (
            self.declare_parameter("control_loop_rate", 0.001)
            .get_parameter_value()
            .double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )
        self.publish_seq = UInt32(0)
        self.frame_id = "map"

        self.dt = self.control_loop_rate

        self.initialized = False
        self.first_loop = True

        # state vector X
        """
        [
            [initial_x],
            [initial_y],
            [v_x],
            [v_y],
            [yaw],
            [omega_z],
        ]
        """
        self.x_est = np.zeros((6, 1))  # estimated state vector

        self.P_est = np.zeros((6, 6))  # estiamted state covariance matrix

        self.x_pred = np.zeros((6, 1))  # Predicted state vector
        self.P_pred = np.zeros((6, 6))  # Predicted state covariance matrix

        # Define state transition matrix
        """
        # [x                ...             ]
        # [y                ...             ]
        # [v_x              ...             ]
        # [x_y              ...             ]
        # [yaw              ...             ]
        # [omega_z          ...             ]
        x = x + v_x * dt
        y = y + v_y * dt
        v_x = v_x
        v_y = v_y
        yaw = yaw + omega_z * dt
        omega_z = omega_z
        """
        self.A = np.array(
            [
                [1, 0, self.dt, 0, 0, 0],
                [0, 1, 0, self.dt, 0, 0],
                [0, 0, 1, 0, 0, 0],
                [0, 0, 0, 1, 0, 0],
                [0, 0, 0, 0, 1, self.dt],
                [0, 0, 0, 0, 0, 1],
            ]
        )

        # Define measurement matrix
        """
        1. GPS: x, y
        2. Velocity: v_x, v_y
        3. IMU: yaw, omega_z
        -> 6 measurements for a state vector of 6
        """
        self.H = np.array(
            [
                [1, 0, 0, 0, 0, 0],  # x
                [0, 1, 0, 0, 0, 0],  # y
                [0, 0, 1, 0, 0, 0],  # v_x
                [0, 0, 0, 1, 0, 0],  # v_y
                [0, 0, 0, 0, 1, 0],  # yaw
                [0, 0, 0, 0, 0, 1],
            ]
        )  # omega_z

        # Define Measurement Variables
        self.z_gps = np.zeros((2, 1))  # GPS measurements (x, y)
        self.z_v = np.zeros((2, 1))  # Velocity measurement (v_x, v_y)
        self.z_imu = np.zeros((2, 1))  # IMU measurements (yaw, omega_z)

        # The process covariance matrix Q is defined as:
        self.Q = np.diag([0.0001, 0.0001, 0.00001, 0.00001, 0.000001, 0.00001])
        # The measurement covariance matrix R is defined as:
        self.R = np.diag([0.0007, 0.0007, 0, 0, 0, 0])

        self.K = np.zeros((6, 6))  # Kalman gain

        self.latitude = 0  # latitude of the current position

        # Subscriber
        # Initialize the subscriber for the OpenDrive Map
        self.map_sub = self.create_subscription(
            String,
            "/carla/" + self.role_name + "/OpenDRIVE",
            self.get_geoRef,
            qos_profile=1,
        )
        # Initialize the subscriber for the IMU Data
        self.imu_subscriber = self.create_subscription(
            Imu,
            "/carla/" + self.role_name + "/IMU",
            self.update_imu_data,
            qos_profile=1,
        )
        # Initialize the subscriber for the GPS Data
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.update_gps_data,
            qos_profile=1,
        )
        # Initialize the subscriber for the unfiltered_pos in XYZ
        self.avg_z = np.zeros((GPS_RUNNING_AVG_ARGS, 1))
        self.avg_gps_counter: int = 0
        self.unfiltered_pos_subscriber = self.create_subscription(
            PoseStamped,
            "/paf/" + self.role_name + "/unfiltered_pos",
            self.update_unfiltered_pos,
            qos_profile=1,
        )
        # Initialize the subscriber for the velocity
        self.velocity_subscriber = self.create_subscription(
            CarlaSpeedometer,
            "/carla/" + self.role_name + "/Speed",
            self.update_velocity,
            qos_profile=1,
        )

        # Publisher
        # Initialize the publisher for the kalman-position
        self.kalman_position_publisher = self.create_publisher(
            PoseStamped, "/paf/" + self.role_name + "/kalman_pos", qos_profile=1
        )
        # Initialize the publisher for the kalman-heading
        self.kalman_heading_publisher = self.create_publisher(
            Float32, "/paf/" + self.role_name + "/kalman_heading", qos_profile=1
        )

        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.create_timer(self.control_loop_rate, self.main_loop)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def main_loop(self):
        """
        Run the Kalman Filter
        """
        # wait until the car receives the first GPS Data
        # from the update_unfiltered_pos method
        if not self.initialized:
            return

        if self.first_loop:
            self.get_logger().info("KalmanFilter started its loop!")
            # initialize the state vector x_est and the covariance matrix P_est
            # initial state vector x_0
            self.x_0 = np.array(
                [
                    [self.z_gps[0, 0]],
                    [self.z_gps[1, 0]],
                    [self.z_v[0, 0]],
                    [self.z_v[1, 0]],
                    [self.z_imu[0, 0]],
                    [self.z_imu[1, 0]],
                ]
            )
            self.x_est = np.copy(self.x_0)  # estimated initial state vector
            self.P_est = np.eye(6) * 1  # estiamted initial state covariance matrix
            self.first_loop = False

        self.predict()
        self.update()

        # Publish the kalman-data:
        self.publish_kalman_heading()
        self.publish_kalman_location()

    def predict(self):
        """
        Predict the next state
        """
        # Predict the next state and covariance matrix, pretending the last
        # velocity state estimate stayed constant
        self.x_pred = self.A @ self.x_est
        self.P_pred = self.A @ self.P_est @ self.A.T + self.Q

    def update(self):
        """
        Update the state
        """
        # Measurementvector z
        z = np.concatenate((self.z_gps, self.z_v, self.z_imu))
        # Measurement residual y
        y = z - self.H @ self.x_pred
        # Residual covariance S
        S = self.H @ self.P_pred @ self.H.T + self.R
        # Kalman gain K
        self.K = self.P_pred @ self.H.T @ np.linalg.inv(S)
        # State estimate x_est
        self.x_est = self.x_pred + self.K @ y
        # State covariance estimate P_est
        # (Joseph form of the covariance update equation)
        self.P_est = (np.eye(6) - self.K @ self.H) @ self.P_pred

    def publish_kalman_heading(self):
        """
        Publish the Kalman heading
        """
        # Initialize the Kalman heading
        kalman_heading = Float32()

        # Set the Kalman heading
        kalman_heading.data = self.x_est[4, 0]

        # Publish the Kalman heading
        self.kalman_heading_publisher.publish(kalman_heading)

    def publish_kalman_location(self):
        """
        Publish the Kalman position
        """

        # Initialize the Kalman position
        kalman_position = PoseStamped()

        # Set the Kalman position
        kalman_position.header.frame_id = self.frame_id
        kalman_position.header.stamp = self.get_clock().now().to_msg()
        kalman_position.header.seq = self.publish_seq

        self.publish_seq.data += 1

        kalman_position.pose.position.x = self.x_est[0, 0]
        kalman_position.pose.position.y = self.x_est[1, 0]

        kalman_position.pose.position.z = self.latitude

        kalman_position.pose.orientation.x = 0
        kalman_position.pose.orientation.y = 0
        kalman_position.pose.orientation.z = 1
        kalman_position.pose.orientation.w = 0
        # Publish the Kalman position
        self.kalman_position_publisher.publish(kalman_position)

    def update_imu_data(self, imu_data):
        """
        Update the IMU Data by:
        - calculating the heading using quaternions
        - calculating the angular velocity
        """
        orientation_x = imu_data.orientation.x
        orientation_y = imu_data.orientation.y
        orientation_z = imu_data.orientation.z
        orientation_w = imu_data.orientation.w

        # Calculate the heading based on the orientation given by the IMU
        data_orientation_q = [
            orientation_x,
            orientation_y,
            orientation_z,
            orientation_w,
        ]

        heading = quat_to_heading(data_orientation_q)

        # update IMU Measurements:
        self.z_imu[0, 0] = heading
        self.z_imu[1, 0] = imu_data.angular_velocity.z

        """
        if imu_data.orientation_covariance[8] != 0:
            # [8] because we want the diag z element of the covariance matrix
            self.R[2, 2] = imu_data.orientation_covariance[8]
        if imu_data.angular_velocity_covariance[8] != 0:
            # [8] because we want the diag z element of the covariance matrix
            self.R[3, 3] = imu_data.angular_velocity_covariance[8]
        """

    def update_gps_data(self, gps_data):
        """
        This function is intended to update the GPS data
        BUT: currently it does nothing
        -> the position is updated by the update_unfiltered_pos function
            -> the unfiltered position is the GPS data converted into x/y/z coordinates
        """
        # look up if covariance type is not 0 (0 = COVARANCE_TYPE_UNKNOWN)
        # (1 = approximated, 2 = diagonal known or 3 = known)
        # if it is not 0 -> update the covariance matrix
        """
        if gps_data.position_covariance_type != 0:
            # [0] because we want the diag lat element of the covariance matrix
            self.R[0, 0] = gps_data.pose.covariance[0]
            # [5] because we want the diag lon element of the covariance matrix
            self.R[1, 1] = gps_data.pose.covariance[5]
        """
        pass

    def update_unfiltered_pos(self, unfiltered_pos):
        """
        Update the current position
        ALSO: allows the Kalman filter to start running
              by setting self.initialized to True
        """
        # Update GPS measurements:
        self.z_gps[0, 0] = unfiltered_pos.pose.position.x
        self.z_gps[1, 0] = unfiltered_pos.pose.position.y

        z = unfiltered_pos.pose.position.z

        self.avg_z = np.roll(self.avg_z, -1, axis=0)
        self.avg_z[-1] = np.matrix([z])
        avg_z = np.mean(self.avg_z, axis=0)

        self.latitude = avg_z

        # Set self.initialized to True so that the Kalman filter can start
        self.initialized = True

    def update_velocity(self, velocity):
        """
        Update the velocity
        using the yaw angle in the estimated state x_est x_est[2] = yaw
        """
        self.z_v[0, 0] = velocity.speed * math.cos(self.x_est[2, 0])
        self.z_v[1, 0] = velocity.speed * math.sin(self.x_est[2, 0])

    def get_geoRef(self, opendrive: String):
        """
        Reads the reference values for lat and lon from the Carla OpenDriveMap
        Args:
            opendrive (String): OpenDrive Map from Carla
        """
        root = eTree.fromstring(opendrive.data)
        header = root.find("header")
        geoRefText = header.find("geoReference").text

        latString = "+lat_0="
        lonString = "+lon_0="

        indexLat = geoRefText.find(latString)
        indexLon = geoRefText.find(lonString)

        indexLatEnd = geoRefText.find(" ", indexLat)
        indexLonEnd = geoRefText.find(" ", indexLon)

        latValue = float(geoRefText[indexLat + len(latString) : indexLatEnd])
        lonValue = float(geoRefText[indexLon + len(lonString) : indexLonEnd])

        CoordinateTransformer.la_ref = latValue
        CoordinateTransformer.ln_ref = lonValue
        CoordinateTransformer.ref_set = True
        self.transformer = CoordinateTransformer()


def main(args=None):
    """
    Main function starts the node
    :param args:
    """
    rclpy.init(args=args)

    try:
        node = KalmanFilter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

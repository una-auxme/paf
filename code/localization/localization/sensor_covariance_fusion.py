#!/usr/bin/env python

"""
Carla publishes the IMU and GPS Data without covariances.

We have to decide on clever values and add them to the data.
This node reads a param file (sensor_covariances.yaml)
and listens to the topics with IMU and GPS data.

When a message is received the covariance is added and the data is sent out again.

The nodes outputs are used by the local and global EKFs and the gps_transform node.
"""

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, NavSatFix
from paf_common.parameters import update_attributes
from rcl_interfaces.msg import (
    ParameterDescriptor,
)


class SensorCovarianceFusion(Node):

    def __init__(self):
        """
        Constructor / Setup
        :return:
        """
        super().__init__("sensor_covariance_fusion")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Parameters
        self.imu_orientation = (
            self.declare_parameter(
                "imu_orientation",
                descriptor=ParameterDescriptor(
                    description="IMU Covariance for Orientation",
                ),
            )
            .get_parameter_value()
            .double_array_value
        )
        self.imu_angular_velocity = (
            self.declare_parameter(
                "imu_angular_velocity",
                descriptor=ParameterDescriptor(
                    description="IMU Covariance for Angular Velocity",
                ),
            )
            .get_parameter_value()
            .double_array_value
        )
        self.imu_linear_acceleration = (
            self.declare_parameter(
                "imu_linear_acceleration",
                descriptor=ParameterDescriptor(
                    description="IMU Covariance for Linear Acceleration",
                ),
            )
            .get_parameter_value()
            .double_array_value
        )
        self.gps_position = (
            self.declare_parameter(
                "gps_position",
                descriptor=ParameterDescriptor(
                    description="Alt,Lat,Long Covariance",
                ),
            )
            .get_parameter_value()
            .double_array_value
        )

        # The publishers (topic names have to coincide with ekf_config.yaml)
        self.imu_publisher = self.create_publisher(Imu, "/imu/data", qos_profile=10)
        self.gps_publisher = self.create_publisher(
            NavSatFix, "/gps/fix", qos_profile=10
        )

        # Initialize the subscriber for the IMU data
        self.imu_subscriber = self.create_subscription(
            Imu,
            "/carla/hero/IMU",
            self.imu_callback,
            qos_profile=1,
        )

        # Initialize the subscriber for the Gps data
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "/carla/hero/GPS",
            self.gps_callback,
            qos_profile=1,
        )

        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def imu_callback(self, imu: Imu):
        """An IMU message is received.

        Adds covariances and sends out again.
        """

        imu.orientation_covariance = self.imu_orientation
        imu.angular_velocity_covariance = self.imu_angular_velocity
        imu.linear_acceleration_covariance = self.imu_linear_acceleration

        self.imu_publisher.publish(imu)

    def gps_callback(self, gps: NavSatFix):
        """GPS data is received.

        Add covariances and sent out again.
        """
        gps.position_covariance = self.gps_position

        self.gps_publisher.publish(gps)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SensorCovarianceFusion()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

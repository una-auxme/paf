#!/usr/bin/env python

"""
This node listens to GPS Data and converts this data into an Odometry message.
The Odometry message is then passed into the global EKF.

It is a substitute for the navsat_transform node (in the robot_localization package)
which only works in UTM coordinates but Carla uses the WGS coordinate system.

This node gets its input from the sensor_covariance_fusion node.
"""

from typing import List
import rclpy
from rclpy.publisher import Publisher
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry
from paf_common.parameters import update_attributes

from localization.coordinate_transformation import CoordinateTransformer


class GpsTransform(Node):

    def __init__(self):
        super().__init__("gps_transform")
        self.get_logger().info(f"{type(self).__name__} node initializing...")
        self.transformer = CoordinateTransformer()
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )

        # Initalize publisher for Odometry data
        self.odometry_publisher: Publisher = self.create_publisher(
            Odometry, "/odometry/gps", qos_profile=10
        )

        # Initialize the subscriber for the GPS data
        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            qos_profile=10,
        )
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def process_data(self, gps: NavSatFix):
        """Transforms GPS data to Odometry message

        Args:
            gps (NavSatFix): GPS Data to process
        """
        out = Odometry()
        out.header = gps.header
        out.header.frame_id = "global"
        out.child_frame_id = self.role_name

        (
            out.pose.pose.position.x,
            out.pose.pose.position.y,
            out.pose.pose.position.z,
        ) = self.transformer.gnss_to_xyz(gps.latitude, gps.longitude, gps.altitude)
        for i in range(3):
            out.pose.covariance[i + (i * 6)] = gps.position_covariance[i + (i * 3)]

        self.odometry_publisher.publish(out)

    def gps_callback(self, gps: NavSatFix):
        self.process_data(gps)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = GpsTransform()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

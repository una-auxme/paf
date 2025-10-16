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
        self.position_use_ground_truth = (
            self.declare_parameter("position_use_ground_truth", False)
            .get_parameter_value()
            .bool_value
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

        self.map = None

        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def process_data(self, gps: NavSatFix):
        """Transforms GPS data to Odometry message

        Args:
            gps (NavSatFix): GPS Data to process
        """
        if self.position_use_ground_truth and self.map is None:
            import carla
            import os

            carla_host = os.environ.get("CARLA_SIM_HOST")
            if carla_host is None:
                self.get_logger().fatal("Environment variable CARLA_SIM_HOST not set!")
                exit(1)
            carla_port = int(os.environ.get("CARLA_PORT", "2000"))
            self.client = carla.Client(carla_host, carla_port)
            self.world = self.client.get_world()
            self.map = self.world.get_map()
            self.carla_car = None
            for actor in self.world.get_actors():
                if actor.attributes.get("role_name") == "hero":
                    self.carla_car = actor
                    break
            if self.carla_car is None:
                self.get_logger().fatal("Actor with role name hero not found!")
                exit(1)

        out = Odometry()
        out.header = gps.header
        out.header.frame_id = "global"
        out.child_frame_id = self.role_name

        if self.position_use_ground_truth and self.carla_car is not None:
            carla_gt_pos = self.carla_car.get_location()
            out.pose.pose.position.x = carla_gt_pos.x
            out.pose.pose.position.y = -carla_gt_pos.y
            out.pose.pose.position.z = carla_gt_pos.z

        else:
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

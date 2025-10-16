import os
from typing import Optional
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped, Pose

import carla

from localization.coordinate_transformation import CoordinateTransformer


class GpsDebug(Node):

    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )

        self.create_subscription(
            NavSatFix,
            "/gps/fix",
            self.gps_callback,
            qos_profile=10,
        )
        self.pose: Optional[Pose] = None
        self.create_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.current_pos_callback,
            qos_profile=10,
        )

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

        self.transformer = CoordinateTransformer()

        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def gps_callback(self, gps: NavSatFix):
        if self.pose is None:
            return

        carla_gt_pos = self.carla_car.get_location()
        carla_gt_gnss = self.map.transform_to_geolocation(location=carla_gt_pos)

        # check_gnss is gnss lat/long calculated with the carla-intern function based
        # on our calculated x/y/z coordinates
        # -> If it matches the gps lat/long, our gnss_to_xyz function is correct
        check_gnss = self.map.transform_to_geolocation(
            location=carla.Location(
                self.pose.position.x,
                self.pose.position.y,
                self.pose.position.z,
            )
        )
        calculated_gt_pos = self.transformer.gnss_to_xyz(
            carla_gt_gnss.latitude, carla_gt_gnss.longitude, carla_gt_gnss.altitude
        )
        self.get_logger().info(
            f"Map geolocation: "
            f"{self.map.transform_to_geolocation(location=carla.Location(0, 0, 0))} \n"
            f"Carla gnss: lat: {gps.latitude}, long: {gps.longitude}, "
            f"alt: {gps.altitude} \n"
            f"Carla calculated pos: {self.pose.position} \n"
            f"Check gnss: {check_gnss} \n"
            f"Carla gt pos: {carla_gt_pos} \n"
            f"Carla gt gnss: {carla_gt_gnss} \n"
            f"Calculated gt pos: {calculated_gt_pos}"
        )

    def current_pos_callback(self, pose: PoseStamped):
        self.pose = pose.pose


def main(args=None):
    from paf_common.debugging import start_debugger

    start_debugger(wait_for_client=False)

    rclpy.init(args=args)

    try:
        node = GpsDebug()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

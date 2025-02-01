#!/usr/bin/env python
from rospy import Publisher
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

from coordinate_transformation import CoordinateTransformer

from typing import Optional


class GPSTransform(CompatibleNode):

    def __init__(self):
        super().__init__("gps_tranasform")

        self.odometry: Optional[Odometry] = None
        self.gps: Optional[NavSatFix] = None

        self.odometry_publisher: Publisher = self.new_publisher(
            Odometry, "/odometry/gps", qos_profile=10
        )

        # Initialize the subscriber for the GPS data
        self.gps_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/hero/GPS",
            self.gps_callback,
            qos_profile=10,
        )

        self.odometry_subscriber = self.new_subscription(
            Odometry, "odometry/filtered", self.odometry_callback, qos_profile=10
        )

        self.transfomer = CoordinateTransformer()

        # self.new_timer(1 / 10.0, lambda _: self.process_data())  # Other might not work

    def process_data(self):
        if self.odometry is None or self.gps is None:
            return
        # Odometry is odometry_global and from global to hero. (we sent dom to heo).
        # GPS is from global to gps
        # Navsat publishes global to ''
        # What do we want?

        out = Odometry()
        out.header = self.odometry.header
        out.header.frame_id = "global"
        out.child_frame_id = "hero"

        out.pose = self.odometry.pose
        (
            out.pose.pose.position.x,
            out.pose.pose.position.y,
            out.pose.pose.position.z,
        ) = self.transfomer.gnss_to_xyz(
            self.gps.latitude, self.gps.longitude, self.gps.altitude
        )

        out.twist = self.odometry.twist
        self.odometry_publisher.publish(out)

        self.odometry = None
        self.gps = None

    def gps_callback(self, gps: NavSatFix):
        self.gps = gps
        if self.odometry is not None:
            self.process_data()

    def odometry_callback(self, odom: Odometry):
        self.odometry = odom
        if self.gps is not None:
            self.process_data()


def main(args=None):
    roscomp.init("forward_imu", args=args)

    try:
        node = GPSTransform()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

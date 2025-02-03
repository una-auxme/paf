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
            "/gps/fix",
            self.gps_callback,
            qos_profile=10,
        )

        self.odometry_subscriber = self.new_subscription(
            Odometry, "odometry/filtered", self.odometry_callback, qos_profile=10
        )

        self.transfomer = CoordinateTransformer()

    def process_data(self):
        if self.gps is None:
            return

        out = Odometry()
        out.header = self.gps.header
        out.header.frame_id = "global"
        out.child_frame_id = "hero"

        (
            out.pose.pose.position.x,
            out.pose.pose.position.y,
            out.pose.pose.position.z,
        ) = self.transfomer.gnss_to_xyz(
            self.gps.latitude, self.gps.longitude, self.gps.altitude
        )
        for i in range(3):
            out.pose.covariance[i + (i * 6)] = self.gps.position_covariance[i + (i * 3)]

        self.odometry_publisher.publish(out)

    def gps_callback(self, gps: NavSatFix):
        self.gps = gps

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

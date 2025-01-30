#!/usr/bin/env python

import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Imu, NavSatFix

"""This Node forward the IMU the other nodes
The """


class ForwardIMU(CompatibleNode):

    def __init__(self):
        """
        Constructor / Setup
        :return:
        """
        super().__init__("forward_imu")

        self.imu_publisher = self.new_publisher(Imu, "/imu/data", qos_profile=10)
        self.gps_publisher = self.new_publisher(NavSatFix, "/gps/fix", qos_profile=10)

        # Initialize the subscriber for the IMU data
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/hero/IMU",
            self.imu_callback,
            qos_profile=1,
        )

        # Initialize the subscriber for the IMU data
        self.imu_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/hero/GPS",
            self.gps_callback,
            qos_profile=1,
        )

    def imu_callback(self, imu: Imu):
        imu.header.frame_id = "hero"  # ??
        imu.linear_acceleration_covariance = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        imu.angular_velocity_covariance = [1, 0, 0, 0, 1, 0, 0, 0, 1]
        imu.orientation_covariance = [1, 0, 0, 0, 1, 0, 0, 0, 0]
        # yaw is 100 % accurate
        self.imu_publisher.publish(imu)
        # doesnt work anyway

    def gps_callback(self, gps: NavSatFix):
        gps.header.frame_id = "hero"
        gps.position_covariance = [1e-3, 0, 0, 0, 1e-3, 0, 0, 0, 1e-3]
        self.gps_publisher.publish(gps)


def main(args=None):
    roscomp.init("forward_imu", args=args)

    try:
        node = ForwardIMU()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

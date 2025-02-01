#!/usr/bin/env python
import rospy
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
        super().__init__("sensor_covariance_fusion")

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
        # imu.header.frame_id = "hero"  #Reduces messages of unknown transform
        imu.orientation_covariance = rospy.get_param("~imu_orientation")
        imu.angular_velocity_covariance = rospy.get_param("~imu_angular_velocity")
        imu.linear_acceleration_covariance = rospy.get_param("~imu_linear_acceleration")
        self.imu_publisher.publish(imu)

    def gps_callback(self, gps: NavSatFix):
        # gps.header.frame_id = "hero"
        gps.position_covariance = rospy.get_param("~gps_position")
        self.gps_publisher.publish(gps)


def main(args=None):
    roscomp.init("sensor_covariance_fusion", args=args)

    try:
        node = ForwardIMU()
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

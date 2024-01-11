#!/usr/bin/env python

"""
This node publishes all relevant topics for the ekf node.
"""
import math
import numpy as np
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from coordinate_transformation import CoordinateTransformer
from tf.transformations import euler_from_quaternion
from xml.etree import ElementTree as eTree
GPS_RUNNING_AVG_ARGS: int = 10


class PositionPublisherNode(CompatibleNode):
    """
    Node publishes a filtered gps signal.
    This is achieved using a rolling average.
    """

    def __init__(self):
        """
        Constructor / Setup
        :return:
        """

        super(PositionPublisherNode, self).__init__('ekf_translation')
        self.loginfo("Position publisher node started")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        # todo: automatically detect town
        self.transformer = None

        # Subscriber
        self.map_sub = self.new_subscription(
            String,
            "/carla/" + self.role_name + "/OpenDRIVE",
            self.get_geoRef,
            qos_profile=1)

        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/IMU",
            self.update_imu_data,
            qos_profile=1)

        self.gps_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.update_gps_data,
            qos_profile=1)

        # Publisher
        # 2D Odometry (Maybe Speedometer?)
        self.ekf_odom_publisher = self.new_publisher(
            Odometry,
            "/odom",
            qos_profile=1)

        # IMU
        self.ekf_imu_publisher = self.new_publisher(
            Imu,
            "/imu_data",
            qos_profile=1)

        self.avg_xyz = np.zeros((GPS_RUNNING_AVG_ARGS, 3))
        self.avg_gps_counter: int = 0
        # 3D Odometry (GPS)
        self.cur_pos_publisher = self.new_publisher(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            qos_profile=1)

        self.__heading: float = 0
        self.__heading_publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            qos_profile=1)

        self.__yaw_publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/yaw",
            qos_profile=1)

    def get_geoRef(self, opendrive: String):
        """_summary_
        Reads the reference values for lat and lon from the carla OpenDriveMap
        Args:
            opendrive (String): OpenDrive Map from carla
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

        latValue = float(geoRefText[indexLat + len(latString):indexLatEnd])
        lonValue = float(geoRefText[indexLon + len(lonString):indexLonEnd])

        CoordinateTransformer.la_ref = latValue
        CoordinateTransformer.ln_ref = lonValue
        CoordinateTransformer.ref_set = True
        self.transformer = CoordinateTransformer()

    def update_imu_data(self, data: Imu):
        """
        This method is called when new IMU data is received.
        It handles all necessary updates and publishes the heading.
        :param data: new IMU measurement
        :return:
        """
        imu_data = Imu()

        imu_data.header.stamp = data.header.stamp
        imu_data.header.frame_id = "hero"

        imu_data.orientation.x = data.orientation.x
        imu_data.orientation.y = data.orientation.y
        imu_data.orientation.z = data.orientation.z
        imu_data.orientation.w = data.orientation.w
        imu_data.orientation_covariance = [0, 0, 0,
                                           0, 0, 0,
                                           0, 0, 0]

        imu_data.angular_velocity.x = data.angular_velocity.x
        imu_data.angular_velocity.y = data.angular_velocity.y
        imu_data.angular_velocity.z = data.angular_velocity.z
        imu_data.angular_velocity_covariance = [0.001, 0, 0,
                                                0, 0.001, 0,
                                                0, 0, 0.001]

        imu_data.linear_acceleration.x = data.linear_acceleration.x
        imu_data.linear_acceleration.y = data.linear_acceleration.y
        imu_data.linear_acceleration.z = data.linear_acceleration.z
        imu_data.linear_acceleration_covariance = [0.001, 0, 0,
                                                   0, 0.001, 0,
                                                   0, 0, 0.015]

        self.ekf_imu_publisher.publish(imu_data)

        # Calculate the heading based on the orientation given by the IMU
        data_orientation_q = [data.orientation.x,
                              data.orientation.y,
                              data.orientation.z,
                              data.orientation.w]

        roll, pitch, yaw = euler_from_quaternion(data_orientation_q)
        raw_heading = math.atan2(roll, pitch)

        # transform raw_heading so that:
        # ---------------------------------------------------------------
        # | 0 = x-axis | pi/2 = y-axis | pi = -x-axis | -pi/2 = -y-axis |
        # ---------------------------------------------------------------
        heading = (raw_heading - (math.pi / 2)) % (2 * math.pi) - math.pi
        self.__heading = heading
        # self.__heading_publisher.publish(self.__heading)
        self.__heading_publisher.publish(yaw)
        self.__yaw_publisher.publish(yaw)

    def update_gps_data(self, data: NavSatFix):
        """
        This method is called when new GNSS data is received.
        The function calculates the average position and then publishes it.
        Measurements are also transformed to global xyz-coordinates
        :param data: GNSS measurement
        :return:
        """
        # Make sure position is only published when reference values have been
        # read from the Map
        if CoordinateTransformer.ref_set is False:
            self.transformer = CoordinateTransformer()
            CoordinateTransformer.ref_set = True
        if CoordinateTransformer.ref_set is True:
            lat = data.latitude
            lon = data.longitude
            alt = data.altitude

            x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)

            self.avg_xyz = np.roll(self.avg_xyz, -1, axis=0)
            self.avg_xyz[-1] = np.matrix([x, y, z])

            avg_x, avg_y, avg_z = np.mean(self.avg_xyz, axis=0)

            cur_pos = PoseStamped()

            cur_pos.header.stamp = data.header.stamp
            cur_pos.header.frame_id = "global"

            cur_pos.pose.position.x = avg_x
            cur_pos.pose.position.y = avg_y
            cur_pos.pose.position.z = avg_z

            cur_pos.pose.orientation.x = 0
            cur_pos.pose.orientation.y = 0
            cur_pos.pose.orientation.z = 1
            cur_pos.pose.orientation.w = 0

            self.cur_pos_publisher.publish(cur_pos)

    def run(self):
        """
        Control loop
        :return:
        """
        self.spin()


def main(args=None):
    """
    main function
    :param args:
    :return:
    """

    roscomp.init("position_publisher_node", args=args)
    try:
        node = PositionPublisherNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python

"""
This node subscibes to unfiltered data, extracts relevant information
from this data and passes this information on to a different topic:
  - IMU data (/carla/hero/IMU) -> /paf/hero/unfiltered_heading
  - GPS data (/carla/hero/GPS) -> /paf/hero/unfiltered_pos

Both signals can be filtered with the available filters:
  - "EKF" (Default)
  - "Kalman"
  - "RunningAvg" -> "None" is used for heading filter
  - "None"
The chosen filter is used to pass its outputs onto the topics:
  - /paf/hero/current_pos
  - /paf/hero/current_heading

The filter is chosen in the localization.launch file.

!!!
When creating a new filter, the corresponding subscriber
(and callback function if message types do not match)
must be added in the constructor for clean modular programming.
!!!

"""

from typing import List
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
import numpy as np
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu

from std_msgs.msg import Float32, String
from localization.coordinate_transformation import CoordinateTransformer
from localization.coordinate_transformation import quat_to_heading
from xml.etree import ElementTree as eTree
from paf_common.parameters import update_attributes
from rcl_interfaces.msg import (
    ParameterDescriptor,
)

GPS_RUNNING_AVG_ARGS: int = 10


class PositionHeadingPublisherNode(Node):

    def __init__(self):
        """
        Constructor / Setup
        :return:
        """

        super().__init__("position_heading_publisher_node")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Configuration parameters
        self.control_loop_rate = (
            self.declare_parameter("control_loop_rate", 0.05)
            .get_parameter_value()
            .double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )
        # Filter used:
        """
        Possible Filters:
        Pos: EKF, Kalman, RunningAvg, None
        Heading: EKF, Kalman, None
        """
        self.pos_filter = (
            self.declare_parameter(
                "pos_filter",
                "EKF",
                descriptor=ParameterDescriptor(
                    description="Options: EKF, Kalman, RunningAvg, None"
                ),
            )
            .get_parameter_value()
            .string_value
        )
        self.heading_filter = (
            self.declare_parameter(
                "heading_filter",
                "EKF",
                descriptor=ParameterDescriptor(
                    description="Options: EKF, Kalman, None"
                ),
            )
            .get_parameter_value()
            .string_value
        )
        self.get_logger().info(
            f"Pos Filter: {self.pos_filter}, Heading Filter: {self.heading_filter}"
        )

        # todo: automatically detect town
        self.transformer = None

        # 3D Odometry (GPS)
        self.avg_xyz = np.zeros((GPS_RUNNING_AVG_ARGS, 3))
        self.avg_gps_counter: int = 0

        # region Subscriber START

        self.map_sub = self.create_subscription(
            String,
            "/carla/" + self.role_name + "/OpenDRIVE",
            self.get_geoRef,
            qos_profile=1,
        )

        self.imu_subscriber = self.create_subscription(
            Imu,
            "/carla/" + self.role_name + "/IMU",
            self.publish_unfiltered_heading,
            qos_profile=1,
        )

        self.gps_subscriber = self.create_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/GPS",
            self.publish_unfiltered_gps,
            qos_profile=1,
        )

        # Create subscribers depending on the filter used
        # Position Filter:
        if self.pos_filter == "EKF":
            self.ekf_pos_subscriber = self.create_subscription(
                PoseStamped,
                "/paf/" + self.role_name + "/ekf_pos",
                self.publish_filter_pos_as_current_pos,
                qos_profile=1,
            )
        elif self.pos_filter == "Kalman":
            self.kalman_pos_subscriber = self.create_subscription(
                PoseStamped,
                "/paf/" + self.role_name + "/kalman_pos",
                self.publish_filter_pos_as_current_pos,
                qos_profile=1,
            )
        elif self.pos_filter == "RunningAvg":
            self.gps_subscriber_for_running_avg = self.create_subscription(
                NavSatFix,
                "/carla/" + self.role_name + "/GPS",
                self.publish_running_avg_pos_as_current_pos,
                qos_profile=1,
            )
        elif self.pos_filter == "None":
            # No additional subscriber needed
            # -> handled by gps_subscriber
            #    (or rather its callback function publish_unfilterd_gps)
            # -> publishes unfilterd GPS signal as current_pos
            pass

        # insert additional elifs for other filters here

        # Heading Filter:
        if self.heading_filter == "EKF":
            self.ekf_heading_subscriber = self.create_subscription(
                Float32,
                "/paf/" + self.role_name + "/ekf_heading",
                self.publish_current_heading,
                qos_profile=1,
            )
        elif self.heading_filter == "Kalman":
            self.kalman_heading_subscriber = self.create_subscription(
                Float32,
                "/paf/" + self.role_name + "/kalman_heading",
                self.publish_current_heading,
                qos_profile=1,
            )
        elif self.heading_filter == "None":
            # No additional subscriber needed
            # -> handled by imu_subscriber
            #    (or rather its callback function publish_unfiltered_heading)
            # -> publishes unfiltered IMU signal as current_heading
            pass

        # insert additional elifs for other filters here

        # endregion Subscriber END

        # region Publisher START
        # Orientation
        self.unfiltered_heading_publisher = self.create_publisher(
            Float32, f"/paf/{self.role_name}/unfiltered_heading", qos_profile=1
        )

        # 3D Odometry (GPS) for filters -> converted to x/y/z coordinates
        self.unfiltered_gps_publisher = self.create_publisher(
            PoseStamped, f"/paf/{self.role_name}/unfiltered_pos", qos_profile=1
        )
        # Publishes current_pos depending on the filter used
        self.cur_pos_publisher = self.create_publisher(
            PoseStamped, f"/paf/{self.role_name}/current_pos", qos_profile=1
        )
        # Publishes current_heading depending on the filter used
        self.__heading: float = 0.0
        self.__heading_publisher = self.create_publisher(
            Float32, f"/paf/{self.role_name}/current_heading", qos_profile=1
        )

        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    # endregion Publisher END

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    # region HEADING FUNCTIONS
    def publish_unfiltered_heading(self, data: Imu):
        """
        This method is called when new IMU data is received.
        It handles all necessary updates and publishes the heading.
        :param data: new IMU measurement
        :return:
        """
        data_orientation_q = [
            data.orientation.x,
            data.orientation.y,
            data.orientation.z,
            data.orientation.w,
        ]

        heading = quat_to_heading(data_orientation_q)

        # In the case of using "None" filter, the heading is
        # published as current heading, since it is not filtered
        if self.heading_filter == "None":
            self.__heading = heading
            self.__heading_publisher.publish(Float32(data=self.__heading))
        else:
            # in each other case the heading is published as unfiltered heading
            # for further filtering in other nodes such as the EKF
            self.unfiltered_heading_publisher.publish(Float32(data=heading))

    def publish_current_heading(self, data: Float32):
        """
        This method is called when new heading data is received.
        It handles all necessary updates and publishes the heading.
        :param data: new heading measurement
        :return:
        """
        self.__heading = data.data
        self.__heading_publisher.publish(Float32(data=self.__heading))

    # insert new heading functions here...

    # endregion HEADING FUNCTIONS END

    # region POSITION FUNCTIONS

    def publish_running_avg_pos_as_current_pos(self, data: NavSatFix):
        """
        This method is called when new GNSS data is received.
        The function calculates the average position and then publishes it.
        Measurements are also transformed to global x/y/z-coordinates
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

            cur_pos.pose.orientation.x = 0.0
            cur_pos.pose.orientation.y = 0.0
            cur_pos.pose.orientation.z = 1.0
            cur_pos.pose.orientation.w = 0.0

            self.cur_pos_publisher.publish(cur_pos)

    def publish_filter_pos_as_current_pos(self, data: PoseStamped):
        """
        This method is called when new filter data is received.
        The function publishes the filtered position as current position
        :param data: PoseStamped
        :return:
        """
        self.cur_pos_publisher.publish(data)

    def publish_unfiltered_gps(self, data: NavSatFix):
        """
        This method is called when new GNSS data is received.
        It publishes the unfiltered GPS data as x/y/z coordinates (PoseStamped).
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

            unfiltered_pos = PoseStamped()

            unfiltered_pos.header.stamp = data.header.stamp
            unfiltered_pos.header.frame_id = "global"

            unfiltered_pos.pose.position.x = x
            unfiltered_pos.pose.position.y = y
            unfiltered_pos.pose.position.z = z

            unfiltered_pos.pose.orientation.x = 0.0
            unfiltered_pos.pose.orientation.y = 0.0
            unfiltered_pos.pose.orientation.z = 1.0
            unfiltered_pos.pose.orientation.w = 0.0

            # In the case of using "None" filter, the pos is
            # published as current pos, since it is not filtered
            if self.pos_filter == "None":
                self.cur_pos_publisher.publish(unfiltered_pos)
            else:
                # in each other case the pos is published as unfiltered pos
                # for further filtering in other nodes such as the EKF
                self.unfiltered_gps_publisher.publish(unfiltered_pos)

    # insert new position functions here...

    # endregion POSITION FUNCTIONS END

    def get_geoRef(self, opendrive: String):
        """_summary_
        Reads the reference values for lat and lon from the carla OpenDriveMap
        This is necessary for the coordinate transformation from GNSS to x/y/z
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

        latValue = float(geoRefText[indexLat + len(latString) : indexLatEnd])
        lonValue = float(geoRefText[indexLon + len(lonString) : indexLonEnd])

        CoordinateTransformer.la_ref = latValue
        CoordinateTransformer.ln_ref = lonValue
        CoordinateTransformer.ref_set = True
        self.transformer = CoordinateTransformer()


def main(args=None):
    """
    main function
    :param args:
    :return:
    """
    rclpy.init(args=args)
    try:
        node = PositionHeadingPublisherNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

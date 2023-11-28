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
from std_msgs.msg import Float32
from coordinate_transformation import CoordinateTransformer, GeoRef
from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray

import carla
import rospy
import pymap3d as pm
# from carla_msgs.msg import CarlaLo

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
        # self.current_pos = PoseStamped()
        self.ideal_current_pos = PoseStamped()
        self.carla_current_pos = PoseStamped()
        self.ideal_heading = Float32()

        self.loginfo("Position publisher node started")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        # todo: automatically detect town
        self.transformer = CoordinateTransformer(GeoRef.TOWN12)

        # Subscriber
        self.imu_subscriber = self.new_subscription(
            Imu,
            "/carla/" + self.role_name + "/Ideal_IMU",
            self.update_imu_data,
            qos_profile=1)

        self.gps_subscriber = self.new_subscription(
            NavSatFix,
            "/carla/" + self.role_name + "/Ideal_GPS",
            self.update_gps_data,
            qos_profile=1)

        # Publisher
        # 2D Odometry (Maybe Speedometer?)
        self.ekf_odom_publisher = self.new_publisher(
            Odometry,
            "/ideal_odom",
            qos_profile=1)

        # IMU
        self.ekf_imu_publisher = self.new_publisher(
            Imu,
            "/ideal_imu_data",
            qos_profile=1)

        self.avg_xyz = np.zeros((GPS_RUNNING_AVG_ARGS, 3))
        self.avg_gps_counter: int = 0
        # 3D Odometry (GPS)
        self.cur_pos_publisher = self.new_publisher(
            PoseStamped,
            f"/paf/{self.role_name}/ideal_current_pos",
            qos_profile=1)

        self.__heading: float = 0
        self.__heading_publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/ideal_current_heading",
            qos_profile=1)

        # Carla API hero car position
        # Get parameters from the launch file
        host = rospy.get_param('~host', 'carla-simulator')
        port = rospy.get_param('~port', 2000)
        timeout = rospy.get_param('~timeout', 100.0)

        # Connect to the CARLA server
        client = carla.Client(host, port)
        client.set_timeout(timeout)

        # Get the world
        self.world = client.get_world()

        # Get the ego vehicle
        self.vehicle = None

        # Publish the location
        self.carla_pos_publisher = self.new_publisher(
            PoseStamped,
            f"/paf/{self.role_name}/carla_current_pos",
            qos_profile=1)

        # Error Publisher
        """publish error distance between current_pos and ideal_corrent_pos &
            current_pos and carla_current_pos:
            # current_pos and ideal_corrent_pos in location_error[0]
            # current_pos and carla_current_pos in location_error[1]
        """
        self.location_error_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/location_error",
            qos_profile=1)

        # Current_pos subscriber:
        self.current_pos_subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.update_location_error,
            qos_profile=1)

        # publish the error between current_heading and ideal_heading
        self.heading_error_publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/heading_error",
            qos_profile=1)

        # Current_heading subscriber:
        self.current_heading_subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.update_heading_error,
            qos_profile=1)

    def update_heading_error(self, data: Float32):
        """
        This method is called when new current_heading data is received.
        """
        current_heading = data.data

        # calculate the error between ideal_imu and imu
        heading_error = self.ideal_heading.data - current_heading
        self.heading_error_publisher.publish(heading_error)

    def update_location_error(self, data: PoseStamped):
        """
        This method is called when new current_pos data is received.
        It handles all necessary updates and publishes the error.
        :param data: new current_pos measurement
        :return:
        """

        error = Float32MultiArray()
        # error.layout.dim = [0, 0]
        error.data = [0, 0]
        # calculate the error between ideal_current_pos and current_pos
        error.data[0] = math.sqrt((
         self.ideal_current_pos.pose.position.x - data.pose.position.x)**2
         + (self.ideal_current_pos.pose.position.y - data.pose.position.y)**2)
        # calculate the error between carla_current_pos and current_pos
        error.data[1] = math.sqrt((
         self.carla_current_pos.pose.position.x - data.pose.position.x)**2
         + (self.carla_current_pos.pose.position.y - data.pose.position.y)**2)

        self.location_error_publisher.publish(error)

    def update_gps_data(self, data: NavSatFix):
        """
        This method is called when new GNSS data is received.
        The function calculates the average position and then publishes it.
        Measurements are also transformed to global xyz-coordinates
        :param data: GNSS measurement
        :return:
        """
        lat = data.latitude
        lon = data.longitude
        alt = data.altitude
        # x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)
        lonref, latref, altref = GeoRef.TOWN12.value
        x, y, z = pm.geodetic2enu(lat, lon, alt,
                                  lonref, latref, altref)
        # find reason for discrepancy
        x *= 0.998
        y *= 1.003

        # self.avg_xyz = np.roll(self.avg_xyz, -1, axis=0)
        # self.avg_xyz[-1] = np.matrix([x, y, z])

        # avg_x, avg_y, avg_z = np.mean(self.avg_xyz, axis=0)

        cur_pos = PoseStamped()

        cur_pos.header.stamp = data.header.stamp
        cur_pos.header.frame_id = "global"

        cur_pos.pose.position.x = x
        cur_pos.pose.position.y = y
        cur_pos.pose.position.z = z

        cur_pos.pose.orientation.x = 0
        cur_pos.pose.orientation.y = 0
        cur_pos.pose.orientation.z = 1
        cur_pos.pose.orientation.w = 0

        self.cur_pos_publisher.publish(cur_pos)
        self.ideal_current_pos = cur_pos

        # also update carla_car_position:
        if self.vehicle is None:
            for actor in self.world.get_actors():
                if actor.attributes.get('role_name') == self.role_name:
                    self.vehicle = actor
                    break

        carla_pos = PoseStamped()
        carla_pos.header.stamp = data.header.stamp
        carla_pos.header.frame_id = "global"

        pos = self.vehicle.get_location()
        carla_pos.pose.position.x = pos.x
        carla_pos.pose.position.y = -pos.y
        carla_pos.pose.position.z = pos.z

        carla_pos.pose.orientation.x = 0
        carla_pos.pose.orientation.y = 0
        carla_pos.pose.orientation.z = 1
        carla_pos.pose.orientation.w = 0

        self.carla_pos_publisher.publish(carla_pos)
        self.carla_current_pos = carla_pos

    def update_imu_data(self, data: Imu):
        """
        This method is called when new IMU data is received.
        The function calculates the average position and then publishes it.
        :param data: IMU measurement
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
        imu_data.angular_velocity_covariance = [0, 0, 0,
                                                0, 0, 0,
                                                0, 0, 0]

        imu_data.linear_acceleration.x = data.linear_acceleration.x
        imu_data.linear_acceleration.y = data.linear_acceleration.y
        imu_data.linear_acceleration.z = data.linear_acceleration.z
        imu_data.linear_acceleration_covariance = [0, 0, 0,
                                                   0, 0, 0,
                                                   0, 0, 0]

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
        self.__heading_publisher.publish(self.__heading)

        self.ideal_heading = Float32(heading)

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

    roscomp.init("position_publisher_node_2", args=args)
    try:
        node = PositionPublisherNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python

"""
This node publishes all relevant topics for the ekf node.
"""
import os
import csv
import math
import numpy as np
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, String
from coordinate_transformation import CoordinateTransformer
from coordinate_transformation import quat_to_heading
# from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from xml.etree import ElementTree as eTree
import rospy


GPS_RUNNING_AVG_ARGS: int = 10
DATA_SAVING_MAX_TIME: int = 45


class SensorFilterDebugNode(CompatibleNode):
    """
    Node publishes a filtered gps signal.
    This is achieved using a rolling average.
    """
    def __init__(self):
        """
        Constructor / Setup
        :return:
        """

        super(SensorFilterDebugNode, self).__init__('ekf_translation')

        self.loginfo("Sensor Filter Debug node started")

        self.kalman_pos = PoseStamped()
        self.kalman_pos_debug_data = Float32MultiArray()
        self.kalman_heading = Float32()
        self.kalman_heading_debug_data = Float32()
        self.current_pos = PoseStamped()
        self.current_pos_debug_data = Float32MultiArray()
        self.current_heading = Float32()
        self.current_heading_debug_data = Float32()
        self.ideal_current_pos = PoseStamped()
        self.carla_current_pos = PoseStamped()
        self.ideal_heading = Float32()
        self.unfiltered_pos = PoseStamped()
        self.unfiltered_pos_debug_data = Float32MultiArray()

        # csv file attributes
        self.csv_x_created = False
        self.csv_file_path_x = ''
        self.csv_y_created = False
        self.csv_file_path_y = ''
        self.csv_heading_created = False
        self.csv_file_path_heading = ''

        self.loginfo("position_heading_publisher_node started")

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        self.transformer = None

    # Subscriber START
        self.map_sub = self.new_subscription(
            String,
            "/carla/" + self.role_name + "/OpenDRIVE",
            self.get_geoRef,
            qos_profile=1)

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

        # Current_pos subscriber:
        self.current_pos_subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.update_location_error,
            qos_profile=1)

        # Current_heading subscriber:
        self.current_heading_subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.update_heading_error,
            qos_profile=1)

        # Kalman_pos subscriber:
        self.kalman_pos_subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/kalman_pos",
            self.kalman_pos_debug,
            qos_profile=1)
        self.kalman_heading_subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/kalman_heading",
            self.kalman_heading_debug,
            qos_profile=1)

        # Unfiltered_pos subscriber:
        self.unfiltered_pos_subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/unfiltered_pos",
            self.unfiltered_pos_debug,
            qos_profile=1)

    # Subscriber END

    # Publisher START
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

        # Publish the carla location
        self.carla_pos_publisher = self.new_publisher(
            PoseStamped,
            f"/paf/{self.role_name}/carla_current_pos",
            qos_profile=1)

        # Error Publisher
        """publish error distance between
            current_pos and ideal_corrent_pos,
            current_pos and carla_current_pos:
            # current_pos and ideal_corrent_pos in location_error[0]
            # current_pos and carla_current_pos in location_error[1]
        """
        self.location_error_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/location_error",
            qos_profile=1)
        # publish the error between current_heading and ideal_heading
        self.heading_error_publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/heading_error",
            qos_profile=1)

        # Publish x and y coordinates of ideal_GPS and carla_pos
        self.ideal_x_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/ideal_x",
            qos_profile=1)
        self.ideal_y_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/ideal_y",
            qos_profile=1)

        # Kalman Debug Publisher
        self.kalman_debug_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/kalman_debug",
            qos_profile=1)
        # Current_pos Debug Publisher
        self.current_pos_debug_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/current_pos_debug",
            qos_profile=1)
        # Unfiltered_pos Debug Publisher
        self.unfiltered_pos_debug_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/unfiltered_pos_debug",
            qos_profile=1)

    # Publisher END

    def unfiltered_pos_debug(self, data: PoseStamped):
        """
        This method is called when new unfiltered_pos data is received.
        It also publishes error distances between
        unfiltered_pos.x and ideal_pos.x,
        unfiltered_pos.y and ideal_pos.y
        unfiltered_pos and ideal_pos:
        # unfiltered_pos and ideal_pos.x in debug.data[0]
        # unfiltered_pos and ideal_pos.y in debug.data[1]
        # unfiltered_pos and ideal_pos in debug.data[2]
        """
        self.unfiltered_pos = data
        debug = Float32MultiArray()

        debug.data = [0, 0, 0]
        debug.data[0] = (self.ideal_current_pos.pose.position.x -
                         self.unfiltered_pos.pose.position.x)
        debug.data[1] = (self.ideal_current_pos.pose.position.y -
                         self.unfiltered_pos.pose.position.y)
        debug.data[2] = math.sqrt((
            self.unfiltered_pos.pose.position.x -
            self.ideal_current_pos.pose.position.x)**2
            + (self.unfiltered_pos.pose.position.y -
                self.ideal_current_pos.pose.position.y)**2)

        self.unfiltered_pos_debug_data = debug
        self.unfiltered_pos_debug_publisher.publish(debug)

    def current_pos_debug(self):
        """
        This method is called when new current_pos data is received.
        It also publishes error distances between
        current_pos.x and ideal_pos.x,
        current_pos.y and ideal_pos.y
        current_pos and ideal_pos:
        # current_pos and ideal_pos.x in debug.data[0]
        # current_pos and ideal_pos.y in debug.data[1]
        # current_pos and ideal_pos in debug.data[2]
        """
        debug = Float32MultiArray()

        debug.data = [0, 0, 0]
        debug.data[0] = (self.ideal_current_pos.pose.position.x -
                         self.current_pos.pose.position.x)
        debug.data[1] = (self.ideal_current_pos.pose.position.y -
                         self.current_pos.pose.position.y)
        debug.data[2] = math.sqrt((
            self.current_pos.pose.position.x -
            self.ideal_current_pos.pose.position.x)**2
            + (self.current_pos.pose.position.y -
                self.ideal_current_pos.pose.position.y)**2)

        self.current_pos_debug_data = debug
        self.current_pos_debug_publisher.publish(debug)

    def kalman_heading_debug(self, data: Float32):
        """
        This method is called when new kalman_heading data is received.
        """
        self.kalman_heading = data
        self.loginfo("kalman_heading: " + str(data))
        debug = Float32()

        debug.data = self.ideal_heading.data - data.data

        self.kalman_heading_debug_data = debug
        self.save_heading_errors()
        # self.kalman_debug_publisher.publish(debug)

    def kalman_pos_debug(self, data: PoseStamped):
        """
        This method is called when new kalman_pos data is received.
        It also publishes error distances between
        kalman_pos.x and ideal_pos.x,
        kalman_pos.y and ideal_pos.y,
        kalman_pos and ideal_pos:
        # kalman_pos.x and ideal_pos.x in debug.data[0]
        # kalman_pos.y and ideal_pos.y in debug.data[1]
        # kalman_pos and ideal_pos in debug.data[2]
        """
        self.kalman_pos = data
        debug = Float32MultiArray()

        debug.data = [0, 0, 0]
        debug.data[0] = (self.ideal_current_pos.pose.position.x
                         - self.kalman_pos.pose.position.x)
        debug.data[1] = (self.ideal_current_pos.pose.position.y
                         - self.kalman_pos.pose.position.y)
        debug.data[2] = math.sqrt((
         self.kalman_pos.pose.position.x -
         self.ideal_current_pos.pose.position.x)**2
         + (self.kalman_pos.pose.position.y -
            self.ideal_current_pos.pose.position.y)**2)

        # TODO: make it more clean!
        self.kalman_pos_debug_data = debug
        self.current_pos_debug()
        self.kalman_debug_publisher.publish(debug)

        # save data in paf23/doc/06_perception/00_Experiments/kalman_datasets
        # for debugging with matplotlib:
        self.save_location_errors()

    def save_location_errors(self):
        """
        This method saves the current location errors in a csv file.
        in the folders of
        paf23/doc/06_perception/00_Experiments/kalman_datasets
        It does this for a limited amount of time.
        """
        # if rospy.get_time() > 45 stop saving data:
        if rospy.get_time() > DATA_SAVING_MAX_TIME:
            self.loginfo("STOPPED SAVING LOCATION DATA")
            return

        # Specify the path to the folder where you want to save the data
        base_path = ('/workspace/code/perception/'
                     'src/00_Experiments/kalman_datasets/')
        folder_path_x = base_path + 'x_error'
        folder_path_y = base_path + 'y_error'
        # Ensure the directories exist
        os.makedirs(folder_path_x, exist_ok=True)
        os.makedirs(folder_path_y, exist_ok=True)

        if self.csv_x_created is False:
            self.csv_file_path_x = create_file(folder_path_x)
            self.csv_x_created = True
        if self.csv_y_created is False:
            self.csv_file_path_y = create_file(folder_path_y)
            self.csv_y_created = True

        with open(self.csv_file_path_x, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([rospy.get_time(),
                             self.ideal_current_pos.pose.position.x,
                             self.kalman_pos.pose.position.x,
                             self.current_pos.pose.position.x,
                             self.unfiltered_pos.pose.position.x,
                             self.kalman_pos_debug_data.data[0],
                             self.current_pos_debug_data.data[0],
                             self.unfiltered_pos_debug_data.data[0]])

        with open(self.csv_file_path_y, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([rospy.get_time(),
                             self.ideal_current_pos.pose.position.y,
                             self.kalman_pos.pose.position.y,
                             self.current_pos.pose.position.y,
                             self.unfiltered_pos.pose.position.y,
                             self.kalman_pos_debug_data.data[1],
                             self.current_pos_debug_data.data[1],
                             self.unfiltered_pos_debug_data.data[1]])

    def save_heading_errors(self):
        """
        This method saves the current heading errors in a csv file.
        in the folders of
        paf23/doc/06_perception/00_Experiments/kalman_datasets
        It does this for a limited amount of time.
        """
        # if rospy.get_time() > 45 stop saving data:
        if rospy.get_time() > DATA_SAVING_MAX_TIME:
            self.loginfo("STOPPED SAVING HEADING DATA")
            return

        # Specify the path to the folder where you want to save the data
        base_path = ('/workspace/code/perception/'
                     'src/00_Experiments/kalman_datasets/')
        folder_path_heading = base_path + 'heading_error'

        # Ensure the directories exist
        os.makedirs(folder_path_heading, exist_ok=True)

        if self.csv_heading_created is False:
            self.csv_file_path_heading = create_file(folder_path_heading)
            self.csv_heading_created = True

        with open(self.csv_file_path_heading, 'a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow([rospy.get_time(),
                             self.ideal_heading.data,
                             self.kalman_heading.data,
                             self.current_heading.data,
                             self.kalman_heading_debug_data.data,
                             self.current_heading_debug_data.data])

    def update_heading_error(self, data: Float32):
        """
        This method is called when new current_heading data is received.
        """
        self.current_heading = data
        self.loginfo("current_heading: " + str(data))

        heading_error = Float32()

        # calculate the error between ideal_imu and imu
        heading_error.data = (self.ideal_heading.data -
                              self.current_heading.data)
        self.heading_error_publisher.publish(heading_error)

        # for savnig data in csv file:
        self.current_heading_debug_data = heading_error

    def update_location_error(self, data: PoseStamped):
        """
        This method is called when new current_pos data is received.
        It handles all necessary updates and publishes the error.
        :param data: new current_pos measurement
        :return:
        """
        self.current_pos = data
        error = Float32MultiArray()

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

        if self.transformer is None:
            self.transformer = CoordinateTransformer()
        x, y, z = self.transformer.gnss_to_xyz(lat, lon, alt)
        # find reason for discrepancy
        # x *= 0.998
        # y *= 1.003

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

        # TODO check if still needed
        # also update carla_car_position:
        # if self.vehicle is None:
        #     for actor in self.world.get_actors():
        #         if actor.attributes.get('role_name') == self.role_name:
        #             self.vehicle = actor
        #             break

        # carla_pos = PoseStamped()
        # carla_pos.header.stamp = data.header.stamp
        # carla_pos.header.frame_id = "global"

        # pos = self.vehicle.get_location()
        # carla_pos.pose.position.x = pos.x
        # carla_pos.pose.position.y = -pos.y
        # carla_pos.pose.position.z = pos.z

        # carla_pos.pose.orientation.x = 0
        # carla_pos.pose.orientation.y = 0
        # carla_pos.pose.orientation.z = 1
        # carla_pos.pose.orientation.w = 0

        # self.carla_pos_publisher.publish(carla_pos)
        # self.carla_current_pos = carla_pos

        # get x and y coordinates of ideal_GPS and carla_pos
        # publish errors between ideal_x and carla_pos.x
        # and ideal_y and carla_pos.y
        # ideal_x = Float32MultiArray()
        # ideal_y = Float32MultiArray()
        # x_error = (
        #     self.ideal_current_pos.pose.position.x
        #     - self.carla_current_pos.pose.position.x
        # )
        # y_error = (
        #     self.ideal_current_pos.pose.position.y
        #     - self.carla_current_pos.pose.position.y
        # )
        # ideal_x.data = [self.ideal_current_pos.pose.position.x,
        #                 self.carla_current_pos.pose.position.x,
        #                 x_error]
        # ideal_y.data = [self.ideal_current_pos.pose.position.y,
        #                 self.carla_current_pos.pose.position.y,
        #                 y_error]

        # self.ideal_x_publisher.publish(ideal_x)
        # self.ideal_y_publisher.publish(ideal_y)

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

        heading = quat_to_heading(data_orientation_q)

        self.__heading = heading
        self.__heading_publisher.publish(self.__heading)

        self.loginfo("heading: " + str(heading))

        self.ideal_heading = Float32(heading)

    def run(self):
        """
        Control loop
        :return:
        """
        self.spin()


def create_file(folder_path):
    '''
    This function creates a new csv file in the folder_path
    in correct sequence looking like data_00.csv, data_01.csv, ...
    and returns the path to the file.
    '''
    i = 0
    while True:
        file_path = f'{folder_path}/data_{str(i).zfill(2)}.csv'
        if not os.path.exists(file_path):
            with open(file_path, 'w', newline=''):
                pass
            return file_path
        i += 1


def main(args=None):
    """
    main function
    :param args:
    :return:
    """

    roscomp.init("position_heading_publisher_node_2", args=args)
    try:
        node = SensorFilterDebugNode()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

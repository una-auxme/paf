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
from std_msgs.msg import Float32
from coordinate_transformation import CoordinateTransformer
from coordinate_transformation import quat_to_heading
# from tf.transformations import euler_from_quaternion
from std_msgs.msg import Float32MultiArray
from xml.etree import ElementTree as eTree
import rospy
import threading
import carla  # remove when publishing to leaderboard!

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

        super(SensorFilterDebugNode, self).__init__('sensor_filter_debug_node')

        # basic info
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", "0.05")

        # carla attributes
        CARLA_HOST = os.environ.get('CARLA_HOST', 'paf23-carla-simulator-1')
        CARLA_PORT = int(os.environ.get('CARLA_PORT', '2000'))
        self.client = carla.Client(CARLA_HOST, CARLA_PORT)
        self.world = self.client.get_world()
        self.world.wait_for_tick()
        self.carla_car = None

        self.set_carla_attributes()

        # Tracked Attributes for Debugging
        self.current_pos = PoseStamped()
        self.current_heading = Float32()
        self.carla_current_pos = PoseStamped()
        self.unfiltered_pos = PoseStamped()
        self.unfiltered_heading = Float32()

        # test_filter attributes for any new filter to be tested
        # default is kalman filter
        self.test_filter_pos = PoseStamped()
        self.test_filter_heading = Float32()

        # csv file attributes/ flags for plots
        self.csv_x_created = False
        self.csv_file_path_x = ''
        self.csv_y_created = False
        self.csv_file_path_y = ''
        self.csv_heading_created = False
        self.csv_file_path_heading = ''

        self.loginfo("Sensor Filter Debug node started")

        # region Subscriber START

        # Current_pos subscriber:
        self.current_pos_subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.set_current_pos,
            qos_profile=1)

        # Current_heading subscriber:
        self.current_heading_subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.set_current_heading,
            qos_profile=1)

        # test_filter_pos subscriber:
        self.test_filter_pos_subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/kalman_pos",
            self.set_test_filter_pos,
            qos_profile=1)
        # test_filter_heading subscriber:
        self.test_filter_heading_subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/kalman_heading",
            self.set_test_filter_heading,
            qos_profile=1)

        # Unfiltered_pos subscriber:
        self.unfiltered_pos_subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/unfiltered_pos",
            self.set_unfiltered_pos,
            qos_profile=1)
        # Unfiltered_heading subscriber:
        self.unfiltered_heading_subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/unfiltered_heading",
            self.set_unfiltered_heading,
            qos_profile=1)

        # endregion Subscriber END

        # region Publisher START

        # ideal carla publisher for easier debug with rqt_plot
        self.carla_heading_publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/carla_current_heading",
            qos_profile=1)

        self.carla_pos_publisher = self.new_publisher(
            PoseStamped,
            f"/paf/{self.role_name}/carla_current_pos",
            qos_profile=1)

        # Error Publisher
        self.position_debug_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/position_debug",
            qos_profile=1)
        self.heading_debug_publisher = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/heading_debug",
            qos_profile=1)

        # endregion Publisher END

    def set_carla_attributes(self):
        """
        This method sets the carla attributes.
        """
        for actor in self.world.get_actors():
            if actor.attributes.get('role_name') == "hero":
                self.carla_car = actor
                break
        if self.carla_car is None:
            self.logwarn("Carla Hero car still none!")
            return
        else:
            self.carla_current_pos = self.carla_car.get_location()
            self.carla_current_heading = (
                                        self.carla_car.get_transform()
                                        .rotation.yaw
                                        )

    def position_debug(self):
        """
        This method is called every loop_rate.
        It publishes and saves:
        carla_current_pos.x,
        carla_current_pos.y,
        current_pos.x,
        current_pos.y,
        test_filter_pos.x
        test_filter_pos.y

        It publishes and saves error distances between:
        carla_current_pos.x and current_pos.x,
        carla_current_pos.y and current_pos.y
        carla_current_pos and current_pos
        carla_current_pos.x and test_filter_pos.x,
        carla_current_pos.y and test_filter_pos.y
        carla_current_pos and test_filter_pos

        # carla_current_pos.x in debug.data[0]
        # carla_current_pos.y in debug.data[1]
        # current_pos.x in debug.data[2]
        # current_pos.y in debug.data[3]
        # test_filter_pos.x in debug.data[4]
        # test_filter_pos.y in debug.data[5]
        # carla_current_pos.x - current_pos.x in debug.data[6]
        # carla_current_pos.y - current_pos.y in debug.data[7]
        # sqrt[(carla_current_pos - current_pos)^2] in debug.data[8]
        # carla_current_pos.x - test_filter_pos.x in debug.data[9]
        # carla_current_pos.y - test_filter_pos.y in debug.data[10]
        # sqrt[(carla_current_pos - test_filter_pos)^2] in debug.data[11]
        """

        if self.carla_car is None:
            self.logwarn("""Carla Hero car still none!
                         Can not record data for position_debug yet.""")
            return

        debug = Float32MultiArray()

        debug.data = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]

        # all x and y coordinates
        debug.data[0] = self.carla_current_pos.x
        debug.data[1] = self.carla_current_pos.y
        debug.data[2] = self.current_pos.pose.position.x
        debug.data[3] = self.current_pos.pose.position.y
        debug.data[4] = self.test_filter_pos.pose.position.x
        debug.data[5] = self.test_filter_pos.pose.position.y

        # error between carla_current_pos and current_pos
        debug.data[6] = (self.carla_current_pos.x
                         - self.current_pos.pose.position.x)
        debug.data[7] = (self.carla_current_pos.y
                         - self.current_pos.pose.position.y)
        debug.data[8] = math.sqrt((self.carla_current_pos.x
                                  - self.current_pos.pose.position.x)**2
                                  + (self.carla_current_pos.y
                                  - self.current_pos.pose.position.y)**2)

        # error between carla_current_pos and test_filter_pos
        debug.data[9] = (self.carla_current_pos.x
                         - self.test_filter_pos.pose.position.x)
        debug.data[10] = (self.carla_current_pos.y
                          - self.test_filter_pos.pose.position.y)
        debug.data[11] = math.sqrt((self.carla_current_pos.x
                                   - self.test_filter_pos.pose.position.x)**2
                                   + (self.carla_current_pos.y
                                   - self.test_filter_pos.pose.position.y)**2)

        self.position_debug_data = debug
        self.position_debug_publisher.publish(debug)
        # for easier debugging with rqt_plot
        self.carla_pos_publisher.publish(self.carla_current_pos)

    def heading_debug(self):
        """
        This method is called every loop_rate.
        It publishes and saves:
        carla_current_heading,
        current_heading,
        test_filter_heading

        It also publishes and saves heading differences between
        carla_current_heading and current_heading,
        carla_current_heading and test_filter_heading
        to make filters comparable.

        # carla_current_heading in debug.data[0]
        # current_heading in debug.data[1]
        # test_filter_heading in debug.data[2]
        # carla_current_heading - current_heading in debug.data[3]
        # carla_current_heading - test_filter_heading in debug.data[4]
        """

        if self.carla_car is None:
            self.logwarn("""Carla Hero car still none!
                         Can not record data for heading_debug yet.""")
            return

        debug = Float32MultiArray()

        debug.data = [0, 0, 0, 0, 0]

        # all heading values
        debug.data[0] = self.carla_current_heading
        debug.data[1] = self.current_heading.data
        debug.data[2] = self.test_filter_heading.data

        # error between carla_current_heading and current_heading
        debug.data[0] = (self.carla_current_heading
                         - self.current_heading.data)

        # error between carla_current_heading and test_filter_heading
        debug.data[1] = (self.carla_current_heading
                         - self.test_filter_heading.data)

        self.heading_debug_data = debug
        self.heading_debug_publisher.publish(debug)

        # for easier debugging with rqt_plot
        self.carla_heading_publisher.publish(self.carla_current_heading)

    # region Subscriber Callbacks
    def set_unfiltered_pos(self, data: PoseStamped):
        """
        This method is called when new unfiltered_pos data is received.
        """
        self.unfiltered_pos = data

    def set_current_pos(self, data: PoseStamped):
        """
        This method is called when new current_pos data is received.
        """
        self.current_pos = data

    def set_current_heading(self, data: Float32):
        """
        This method is called when new current_heading data is received.
        """
        self.current_heading = data

    def set_test_filter_pos(self, data: PoseStamped):
        """
        This method is called when new test_filter_pos data is received.
        """
        self.test_filter_pos = data

    def set_test_filter_heading(self, data: Float32):
        """
        This method is called when new test_filter_heading data is received.
        """
        self.test_filter_heading = data

    # endregion Subscriber Callbacks

    # region CSV data save methods
    def save_position_errors(self):
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

    # endregion CSV data save methods

    def run(self):
        """
        Main loop of the node:
        - updates carla attributes
        - publishes position_debug and heading_debug
        - saves position and heading errors in csv files (if uncommented)
        :return:
        """
        def loop():
            """
            Loop for the data gathering
            """
            while True:
                # update carla attributes
                self.set_carla_attributes()

                # if carla_car still not found -> skip
                if self.carla_car is None:
                    self.logwarn("""Carla Hero car still none!
                                 Can not record data for debug yet.""")
                    continue

                # update & publish pos debug and heading debug
                self.position_debug()
                self.heading_debug()

                # save debug data in csv files
                self.save_position_errors()
                self.save_heading_errors()

                rospy.sleep(self.control_loop_rate)

        threading.Thread(target=loop).start()
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

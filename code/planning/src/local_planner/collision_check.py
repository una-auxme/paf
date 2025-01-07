#!/usr/bin/env python
# import rospy
import numpy as np
import rospy

# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Subscriber

from carla_msgs.msg import CarlaSpeedometer  # , CarlaWorldInfo

from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Bool
from utils import filter_vision_objects, calculate_rule_of_thumb


class CollisionCheck(CompatibleNode):
    """
    This is currently a test node. In the future this node will be
    responsible for detecting collisions and reporting them.
    """

    def __init__(self):
        super(CollisionCheck, self).__init__("CollisionCheck")
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        # Subscriber for current speed
        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1,
        )
        # Subscriber for lidar objects
        self.lidar_dist = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/Center/object_distance",
            self.__set_all_distances,
            qos_profile=1,
        )
        # Publisher for emergency stop
        self.emergency_pub = self.new_publisher(
            Bool, f"/paf/{self.role_name}/emergency", qos_profile=1
        )
        # Publisher for distance to collision
        self.collision_pub = self.new_publisher(
            Float32MultiArray, f"/paf/{self.role_name}/collision", qos_profile=1
        )
        # Publisher for distance to oncoming traffic
        self.oncoming_pub = self.new_publisher(
            Float32, f"/paf/{self.role_name}/oncoming", qos_profile=1
        )
        # Variables to save vehicle data
        self.__current_velocity: float = None
        self.__object_first_position: tuple = None
        self.__object_last_position: tuple = None
        self.__last_position_oncoming: tuple = None
        self.__first_position_oncoming: tuple = None
        self.logdebug("CollisionCheck started")

    def __set_all_distances(self, data: Float32MultiArray):
        """Callback for lidar ibjects subscriber. Initiates collision check
           for objects in front and oncoming traffic

        Args:
            data (Float32MultiArray): Message from lidar with distance objects
        """
        # Get distance to objects in front
        self.__set_distance(data)
        # Get distance to oncoming traffic
        self.__set_distance_oncoming(data)

    def update_distance(self, reset):
        """Updates distance to the obstacle in front or oncoming traffic.
           Reset determines if the distance should be reset or only updated.

        Args:
            reset (Bool): True: Reset distance to obstacle in front; False:
            Update distance
        """
        if reset:
            # Reset all values if we do not have car in front
            self.__object_last_position = None
            self.__object_first_position = None
            # Signal to ACC to reset speed and distance
            data = Float32MultiArray(data=[np.Inf, np.Inf])
            self.collision_pub.publish(data)
            return
        if self.__object_first_position is None:
            # Update distance, store second distance value for speed
            self.__object_first_position = self.__object_last_position
            self.__object_last_position = None
            return

    def __set_distance(self, data: Float32MultiArray):
        """Filters objects and saves last distance from LIDAR
           for objects in front.
           Afterwards initiates speed calculation

        Args:
            data (Float32): Message from lidar with distance objects
        """
        # Filter onjects in front
        nearest_object = filter_vision_objects(data.data, False)
        if (
            nearest_object is None
            and self.__object_last_position is not None
            and rospy.get_rostime() - self.__object_last_position[0] > rospy.Duration(2)
        ):
            # If no object is in front and last object is older than 2 seconds
            # we assume no object is in front
            self.update_distance(True)
            return
        elif nearest_object is None:
            # If no object is in front abort
            return
        self.__object_last_position = (rospy.get_rostime(), nearest_object[1])
        # Update distance if this is first object since reset
        self.update_distance(False)
        # Calculate speed of object in front and check collision
        self.calculate_obstacle_speed()

    def __set_distance_oncoming(self, data: Float32MultiArray):
        """Filters objects and saves last distance from LIDAR for oncoming
            traffic

        Args:
            data (Float32): Message from lidar with distance objects
        """
        # Filter for oncoming traffic objects
        nearest_object = filter_vision_objects(data.data, True)
        if (
            nearest_object is None
            and self.__last_position_oncoming is not None
            and rospy.get_rostime() - self.__last_position_oncoming[0]
            > rospy.Duration(2)
        ):
            # If no oncoming traffic found and last object is older than 2
            # seconds we assume no object is in front
            self.update_distance_oncoming(True)
            return
        elif nearest_object is None:
            # If no oncoming traffic abort
            return

        self.__last_position_oncoming = (rospy.get_rostime(), nearest_object[1])
        # Update oncoming traffic distance if this is first object since reset
        self.update_distance_oncoming(False)
        # Publish oncoming traffic to Decision Making
        self.oncoming_pub.publish(Float32(data=nearest_object[1]))

    def update_distance_oncoming(self, reset):
        """Updates the distance to the oncoming traffic. Reset determines if
        the distance should be reset or only updated.

        Args:
            reset (Bool): True: Reset distance to oncoming traffic
                            False: Update distance
        """
        if reset:
            # Reset all values if we do not have car in front
            self.__last_position_oncoming = None
            self.__first_position_oncoming = None
            # Publish np.inf to Decision Making
            self.oncoming_pub.publish(Float32(data=np.inf))
            return
        if self.__first_position_oncoming is None:
            self.__first_position_oncoming = self.__last_position_oncoming
            self.__last_position_oncoming = None
            return

    def calculate_obstacle_speed(self):
        """Caluclate the speed of the obstacle in front of the ego vehicle
        based on the distance between to timestamps.
        Then check for collision
        """
        # Check if current speed from vehicle is not None
        if (
            self.__current_velocity is None
            or self.__object_first_position is None
            or self.__object_last_position is None
        ):
            return
        # Calculate time since last position update
        rospy_time_difference = (
            self.__object_last_position[0] - self.__object_first_position[0]
        )
        # Use nanoseconds for time difference to be more accurate
        # and reduce error
        time_difference = rospy_time_difference.nsecs / 1e9
        # Calculate distance (in m)
        distance = self.__object_last_position[1] - self.__object_first_position[1]
        try:
            # Speed difference is distance/time (m/s)
            relative_speed = distance / time_difference
        except ZeroDivisionError:
            # If time difference is 0, we cannot calculate speed
            return
        # Calculate speed of obstacle in front
        speed = self.__current_velocity + relative_speed
        if speed < 0:
            speed = 0
        # Publish speed to ACC for permanent distance check
        # self.speed_publisher.publish(Float32(data=speed))
        # Check for crash
        self.check_crash((self.__object_last_position[1], speed))
        # Update first position to calculate speed when next object is detected
        self.__object_first_position = self.__object_last_position

    def __get_current_velocity(
        self,
        data: CarlaSpeedometer,
    ):
        """Saves current velocity of the ego vehicle

        Args:
            data (CarlaSpeedometer): Message from carla with current speed
        """
        self.__current_velocity = float(data.speed)

    def time_to_collision(self, obstacle_speed, distance):
        """calculates the time to collision with the obstacle in front

        Args:
            obstacle_speed (float): Speed from obstacle in front
            distance (float): Distance to obstacle in front

        Returns:
            float: Time until collision with obstacle in front
        """
        if (self.__current_velocity - obstacle_speed) == 0:
            # If the speed difference is 0, we cannot calculate time
            return -1
        return distance / (self.__current_velocity - obstacle_speed)

    def check_crash(self, obstacle):
        """Checks if and when the ego vehicle will crash
            with the obstacle in front

        Args:
            obstacle (tuple): tuple with distance and
                                speed from obstacle in front
        """
        distance, obstacle_speed = obstacle

        collision_time = self.time_to_collision(obstacle_speed, distance)
        # Calculate emergency distance based on current speed
        emergency_distance = calculate_rule_of_thumb(True, self.__current_velocity)
        if collision_time > 0:
            # If time to collision is positive, a collision is ahead
            if distance < emergency_distance:
                # If distance is smaller than emergency distance,
                # publish emergency brake
                self.emergency_pub.publish(True)
            # Publish collision data to Decision Making and ACC
            data = Float32MultiArray(data=[distance, obstacle_speed])
            self.collision_pub.publish(data)
        else:
            # If no collision is ahead publish np.Inf
            data = Float32MultiArray(data=[np.Inf, obstacle_speed])
            self.collision_pub.publish(data)

    def run(self):
        """
        Control loop
        :return:
        """
        self.spin()


if __name__ == "__main__":
    """
    main function starts the CollisionCheck node
    :param args:
    """
    roscomp.init("CollisionCheck")

    try:
        node = CollisionCheck()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

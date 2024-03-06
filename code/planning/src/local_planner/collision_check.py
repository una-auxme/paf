#!/usr/bin/env python
# import rospy
import numpy as np
import rospy
# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Subscriber
from carla_msgs.msg import CarlaSpeedometer   # , CarlaWorldInfo
# from std_msgs.msg import String
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Bool


class CollisionCheck(CompatibleNode):
    """
    This is currently a test node. In the future this node will be
    responsible for detecting collisions and reporting them.
    """

    def __init__(self):
        super(CollisionCheck, self).__init__('CollisionCheck')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.lidar_position_offset = 2
        # self.obstacle_sub: Subscriber = self.new_subscription(
        # )
        # Subscriber for current speed
        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)
        # Subscriber for lidar distance
        # TODO: Change to real lidar distance
        self.lidar_dist = self.new_subscription(
            Float32,
            f"/carla/{self.role_name}/LIDAR_range",
            self.__set_distance,
            qos_profile=1)
        # Publisher for emergency stop
        self.emergency_pub = self.new_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=1)
        # Publisher for distance to collision
        self.collision_pub = self.new_publisher(
            Float32MultiArray,
            f"/paf/{self.role_name}/collision",
            qos_profile=1)
        # Approx speed publisher for ACC
        self.speed_publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/cc_speed",
            qos_profile=1)
        # Variables to save vehicle data
        self.__current_velocity: float = None
        self.__object_first_position: tuple = None
        self.__object_last_position: tuple = None
        self.logdebug("CollisionCheck started")

    def update_distance(self):
        """Updates the distance to the obstacle in front
        """
        if np.isinf(self.__object_last_position[1]):
            self.__object_last_position = None
            self.__object_first_position = None
            return
        if self.__object_first_position is None:
            self.__object_first_position = self.__object_last_position
            self.__object_last_position = None
            return

    def __set_distance(self, data: Float32):
        """Saves last distance from  LIDAR

        Args:
            data (Float32): Message from lidar with distance
        """
        if np.isinf(data.data) and \
                self.__object_last_position is not None and \
                rospy.get_rostime() - self.__object_last_position[0] < \
                rospy.Duration(1):
            return
        # Set distance - 2 to clear distance from car
        self.__object_last_position = (rospy.get_rostime(), data.data)
        self.update_distance()
        self.calculate_obstacle_speed()

    def calculate_obstacle_speed(self):
        """Caluclate the speed of the obstacle in front of the ego vehicle
            based on the distance between to timestamps
        """
        # Check if current speed from vehicle is not None
        if self.__current_velocity is None or \
                self.__object_first_position is None or \
                self.__object_last_position is None:
            return
        # If distance is np.inf no car is in front

        # Calculate time since last position update
        rospy_time_difference = self.__object_last_position[0] - \
            self.__object_first_position[0]
        time_difference = rospy_time_difference.nsecs/1e9
        # Calculate distance (in m)
        distance = self.__object_last_position[1] - \
            self.__object_first_position[1]
        try:
            # Speed is distance/time (m/s)
            relative_speed = distance/time_difference
        except ZeroDivisionError:
            return

        speed = self.__current_velocity + relative_speed
        # Publish speed to ACC for permanent distance check
        self.speed_publisher.publish(Float32(data=speed))
        # Check for crash
        self.check_crash((self.__object_last_position[1], speed))
        self.__object_first_position = self.__object_last_position

    def __get_current_velocity(self, data: CarlaSpeedometer,):
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
            return -1
        return distance / (self.__current_velocity - obstacle_speed)

    def meters_to_collision(self, obstacle_speed, distance):
        """Calculates the meters until collision with the obstacle in front

        Args:
            obstacle_speed (float): speed from obstacle in front
            distance (float): distance from obstacle in front

        Returns:
            float: distance (in meters) until collision with obstacle in front
        """
        return self.time_to_collision(obstacle_speed, distance) * \
            self.__current_velocity

    @staticmethod
    def calculate_rule_of_thumb(emergency, speed):
        """Calculates the rule of thumb as approximation
        for the braking distance

        Args:
            emergency (bool): if emergency brake is initiated

        Returns:
            float: distance calculated with rule of thumb
        """
        reaction_distance = speed
        braking_distance = (speed * 0.36)**2
        if emergency:
            return reaction_distance + braking_distance / 2
        else:
            return reaction_distance + braking_distance

    def check_crash(self, obstacle):
        """ Checks if and when the ego vehicle will crash
            with the obstacle in front

        Args:
            obstacle (tuple): tuple with distance and
                                speed from obstacle in front
        """
        distance, obstacle_speed = obstacle

        collision_time = self.time_to_collision(obstacle_speed, distance)
        # collision_meter = self.meters_to_collision(obstacle_speed, distance)
        # safe_distance2 = self.calculate_rule_of_thumb(False)
        emergency_distance2 = self.calculate_rule_of_thumb(
            True, self.__current_velocity)
        if collision_time > 0:
            if distance < emergency_distance2:
                # Initiate emergency brake
                self.logerr("Emergency Brake")
                self.emergency_pub.publish(True)
            # When no emergency brake is needed publish collision object
            data = Float32MultiArray(data=[distance, obstacle_speed])
            self.logerr("Collision published")
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
        # def loop(timer_event=None):
        #     """
        #     Checks if distance to a possible object is too small and
        #     publishes the desired speed to motion planning
        #     """
        #     self.update_distance()
        #     self.calculate_obstacle_speed()
        # self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    """
    main function starts the CollisionCheck node
    :param args:
    """
    roscomp.init('CollisionCheck')

    try:
        node = CollisionCheck()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

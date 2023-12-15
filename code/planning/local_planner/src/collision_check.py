#!/usr/bin/env python
# import rospy
import numpy as np
# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Subscriber
from geometry_msgs.msg import PoseStamped
# from carla_msgs.msg import CarlaSpeedometer   # , CarlaWorldInfo
# from std_msgs.msg import String
from std_msgs.msg import Float32, Float32MultiArray
from std_msgs.msg import Bool
import time


class CollisionCheck(CompatibleNode):
    """
    This is currently a test node. In the future this node will be
    responsible for detecting collisions and reporting them.
    """

    def __init__(self):
        super(CollisionCheck, self).__init__('CollisionCheck')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.current_speed = 50 / 3.6  # m/ss
        # TODO: Add Subscriber for Speed and Obstacles
        self.logerr("CollisionCheck started")

        # self.obstacle_sub: Subscriber = self.new_subscription(
        # )
        # Subscriber for current speed
        self.velocity_sub: Subscriber = self.new_subscription(
            Float32,  # CarlaSpeedometer # f"/carla/{self.role_name}/Speed"
            f"/paf/{self.role_name}/test_speed",
            self.__get_current_velocity,
            qos_profile=1)
        # Subscriber for current position
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)
        # Subscriber for lidar distance
        self.lidar_dist = self.new_subscription(
            Float32,
            f"/carla/{self.role_name}/lidar_dist_dev",
            self.calculate_obstacle_speed,
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
        # Variables to save vehicle data
        self.__current_velocity: float = None
        self.__object_last_position: tuple = None
        self._current_position: tuple = None

    def calculate_obstacle_speed(self, new_dist: Float32):
        """Caluclate the speed of the obstacle in front of the ego vehicle
            based on the distance between to timestamps

        Args:
            new_position (Float32): new position received from the lidar
        """
        # Check if current speed from vehicle is not None
        if self.__current_velocity is None:
            self.logerr("Current Speed is None")
            return
        # Check if this is the first time the callback is called
        self.logerr("distance recieved: " + str(new_dist.data))
        if self.__object_last_position is None and \
                new_dist.data is not np.inf:
            self.__object_last_position = (time.time(),
                                           new_dist.data)
            self.logerr("First Position")
            return

        # If distance is np.inf no car is in front
        if new_dist.data is np.inf:
            self.__object_last_position = None
            return
        # Check if too much time has passed since last position update
        if self.__object_last_position[0] + \
                0.5 < time.time():
            self.__object_last_position = (time.time(),
                                           new_dist.data)
            self.logerr("Time difference too big")
            return
        # Calculate time since last position update
        current_time = time.time()
        time_difference = current_time-self.__object_last_position[0]

        # Calculate distance (in m)
        distance = new_dist.data - self.__object_last_position[1]

        # Speed is distance/time (m/s)
        self.logerr("Time Difference: " + str(time_difference))
        self.logerr("Distance: " + str(distance))
        relative_speed = distance/time_difference
        self.logerr("Relative Speed: " + str(relative_speed))
        self.logerr("Current Speed: " + str(self.__current_velocity))
        speed = self.__current_velocity + relative_speed
        self.logerr("Speed: " + str(speed))

        # Check for crash
        self.check_crash((distance, speed))
        self.__object_last_position = (current_time, new_dist.data)

    def __get_current_velocity(self, data: Float32):
        """Saves current velocity of the ego vehicle

        Args:
            data (CarlaSpeedometer): Message from carla with current speed
        """
        self.__current_velocity = float(data.data)

    def __current_position_callback(self, data: PoseStamped):
        """Saves current position of the ego vehicle

        Args:
            data (PoseStamped): Message from Perception with current position
        """
        self._current_position = (data.pose.position.x, data.pose.position.y)

    def time_to_collision(self, obstacle_speed, distance):
        """calculates the time to collision with the obstacle in front

        Args:
            obstacle_speed (float): Speed from obstacle in front
            distance (float): Distance to obstacle in front

        Returns:
            float: Time until collision with obstacle in front
        """
        return distance / (self.current_speed - obstacle_speed)

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

    def calculate_rule_of_thumb(self, emergency):
        """Calculates the rule of thumb as approximation
        for the braking distance

        Args:
            emergency (bool): if emergency brake is initiated

        Returns:
            float: distance calculated with rule of thumb
        """
        reaction_distance = self.current_speed
        braking_distance = (self.current_speed * 0.36)**2
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
        collision_meter = self.meters_to_collision(obstacle_speed, distance)

        # safe_distance2 = self.calculate_rule_of_thumb(False)
        emergency_distance2 = self.calculate_rule_of_thumb(False)

        if collision_time > 0:
            if distance < emergency_distance2:
                # Initiate emergency brake
                self.emergency_pub.publish(True)
                self.logerr("Emergency Brake")
                return
            # When no emergency brake is needed publish collision distance for
            # ACC and Behaviour tree
            data = Float32MultiArray(data=[collision_meter, obstacle_speed])
            self.collision_pub.publish(data)
            self.logerr("Collision Distance: " + str(collision_meter))
            # print(f"Safe Distance Thumb: {safe_distance2:.2f}")
        else:
            # If no collision is ahead publish np.Inf
            data = Float32MultiArray(data=[np.Inf, -1])
            self.collision_pub.publish(data)
            self.logerr("No Collision ahead")

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
    roscomp.init('CollisionCheck')

    try:
        node = CollisionCheck()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

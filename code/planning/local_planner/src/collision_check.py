#!/usr/bin/env python
import rospy
import numpy as np
# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Subscriber
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaSpeedometer   # , CarlaWorldInfo
from nav_msgs.msg import Path
# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray


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
        self.logdebug("CollisionCheck started")

        # self.obstacle_sub: Subscriber = self.new_subscription(
        # )
        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1)

        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)

        self.__current_velocity: float = None
        self.__object_last_position: tuple = None
        self._current_position: tuple= None

    def calculate_obstacle_speed(self, new_position):
        # Calculate time since last position update
        current_time = rospy.get_rostime()
        time_difference = current_time-self.__object_last_position[0]

        # Calculate distance (in m) - Euclidian distance is used as approx
        distance = np.linalg.norm(
            new_position - self.__object_last_position[1])

        # Speed is distance/time (m/s)
        return distance/time_difference

    def __get_current_velocity(self, data: CarlaSpeedometer):
        self.__current_velocity = float(data.speed)
        self.velocity_pub.publish(self.__current_velocity)

    def __current_position_callback(self, data: PoseStamped):
        self._current_position = (data.pose.position.x, data.pose.position.y)

    def time_to_collision(self, obstacle_speed, distance):
        return distance / (self.current_speed - obstacle_speed)

    def meters_to_collision(self, obstacle_speed, distance):
        return self.time_to_collision(obstacle_speed, distance) * \
            self.current_speed

    def calculate_rule_of_thumb(self, emergency):
        reaction_distance = self.current_speed
        braking_distance = (self.current_speed * 0.36)**2
        if emergency:
            return reaction_distance + braking_distance / 2
        else:
            return reaction_distance + braking_distance

    def check_crash(self, obstacle):
        distance, obstacle_speed = obstacle

        collision_time = self.time_to_collision(obstacle_speed, distance)
        collision_meter = self.meters_to_collision(obstacle_speed, distance)

        safe_distance2 = self.calculate_rule_of_thumb(False)
        emergency_distance2 = self.calculate_rule_of_thumb(True)

        # TODO: Convert to Publishers
        if collision_time > 0:
            if distance < emergency_distance2:
                print(f"Emergency Brake needed, {emergency_distance2:.2f}")
            print(f"Ego reaches obstacle after {collision_time:.2f} seconds.")
            print(f"Ego reaches obstacle after {collision_meter:.2f} meters.")
            print(f"Safe Distance Thumb: {safe_distance2:.2f}")
        else:
            print("Ego slower then car in front")

    def run(self):
        """
        Control loop
        :return:
        """

        def loop(timer_event=None):
            pass

        self.new_timer(self.control_loop_rate, loop)
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

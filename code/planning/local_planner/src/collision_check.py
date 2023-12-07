#!/usr/bin/env python
import rospy
import numpy as np
# import tf.transformations
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber, Duration
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from carla_msgs.msg import CarlaRoute, CarlaSpeedometer   # , CarlaWorldInfo
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

        self.speed_limit_OD_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/speed_limits_OpenDrive",
            self.__set_speed_limits_opendrive,
            qos_profile=1)

        # needed to prevent the car from driving before a path to follow is
        # available. Might be needed later to slow down in curves
        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_trajectory,
            qos_profile=1)

        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)

        self.__speed_limits_OD: [float] = []
        self.__trajectory: Path = None
        self.__current_wp_index: int = 0
        self.__current_velocity: float = None
        self.__object_last_position: tuple = None

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

    def __set_trajectory(self, data: Path):
        self.__trajectory = data

    def __set_speed_limits_opendrive(self, data: Float32MultiArray):
        self.__speed_limits_OD = data.data

    def __current_position_callback(self, data: PoseStamped):
        if len(self.__speed_limits_OD) < 1 or self.__trajectory is None:
            return

        agent = data.pose.position
        current_wp = self.__trajectory.poses[self.__current_wp_index].\
            pose.position
        next_wp = self.__trajectory.poses[self.__current_wp_index + 1].\
            pose.position

        # distances from agent to current and next waypoint
        d_old = abs(agent.x - current_wp.x) + abs(agent.y - current_wp.y)
        d_new = abs(agent.x - next_wp.x) + abs(agent.y - next_wp.y)
        if d_new < d_old:
            # update current waypoint and corresponding speed limit
            self.__current_wp_index += 1
            self.__speed_limit = \
                self.__speed_limits_OD[self.__current_wp_index]

    def update(self, speed):
        self.current_speed = speed

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
            if self.__velocity is None:
                self.logdebug("ACC hasn't received the velocity of the ego "
                              "vehicle yet and can therefore not publish a "
                              "velocity")
                return

            if self.__dist < 0.5:
                self.velocity_pub.publish(0)
                self.logwarn("ACC off")
                self.__on = False
                self.__dist = None  # to check if new dist was published
                return

            # Use for testing
            # self.d_dist_pub.publish(self.calculate_optimal_dist()-self.__dist)
            # self.velocity_pub.publish(v)

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

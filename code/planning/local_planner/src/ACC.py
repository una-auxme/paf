#!/usr/bin/env python
import numpy as np
import ros_compatibility as roscomp
# import tf.transformations
from ros_compatibility.node import CompatibleNode
from rospy import Subscriber, Publisher
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaSpeedometer   # , CarlaWorldInfo
from nav_msgs.msg import Path
# from std_msgs.msg import String
from std_msgs.msg import Float32MultiArray, Float32
from collision_check import CollisionCheck


class ACC(CompatibleNode):
    """
    This node recieves a possible collision and
    """

    def __init__(self):
        super(ACC, self).__init__('ACC')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.current_speed = 50 / 3.6  # m/ss

        self.logerr("ACC started")
        # TODO: Add Subscriber for Obsdacle from Collision Check
        self.collision_sub = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/collision",
            self.__get_collision,
            qos_profile=1)

        # Get current speed
        self.velocity_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/test_speed",
            self.__get_current_velocity,
            qos_profile=1)

        # Get initial set of speed limits
        self.speed_limit_OD_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/speed_limits_OpenDrive",
            self.__set_speed_limits_opendrive,
            qos_profile=1)

        # Get trajectory to determine current speed limit
        self.trajectory_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_trajectory,
            qos_profile=1)

        # Get current position to determine current speed limit
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)

        # Publish desiored speed to acting
        self.velocity_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/acc_velocity",
            qos_profile=1)

        # List of all speed limits, sorted by waypoint index
        self.__speed_limits_OD: [float] = []
        # Current Trajectory
        self.__trajectory: Path = None
        # Current index from waypoint
        self.__current_wp_index: int = 0
        # Current speed
        self.__current_velocity: float = None
        # Is an obstacle ahead where we would collide with?
        self.collision_ahead: bool = False
        # Distance and speed from possible collsion object
        self.obstacle: tuple = None
        # Current speed limit
        self.speed_limit: float = None  # m/s

    def __get_collision(self, data: Float32MultiArray):
        """Check if collision is ahead

        Args:
            data (Float32MultiArray): Distance and speed from possible
                                        collsion object
        """
        if np.isinf(data.data[0]):
            # No collision ahead
            self.collision_ahead = False
            self.logerr("No Collision ahead -> ACC")
        else:
            # Collision ahead
            self.collision_ahead = True
            self.obstacle = (data.data[0], data.data[1])
            target_speed = self.calculate_safe_speed()
            if target_speed is not None:
                self.velocity_pub.publish(target_speed)

    def calculate_safe_speed(self):
        """calculates the speed to meet the desired distance to the object

        Returns:
            float: safe speed tp meet the desired distance
        """
        # No speed or obstacle recieved yet
        if self.__current_velocity is None:
            return None
        if self.obstacle is None:
            return None
        # Calculate safety distance
        safety_distance = CollisionCheck.calculate_rule_of_thumb(
            False, self.__current_velocity)
        self.logerr("Safety Distance: " + str(safety_distance))
        if self.obstacle[0] < safety_distance:
            # If safety distance is reached, we want to reduce the speed to
            # meet the desired distance
            # Speed is reduced by the factor of the distance to the safety
            # distance
            # Another solution could be
            # object_speed - (safety_distance-distance)

            safe_speed = self.obstacle[1] * (self.obstacle[0] /
                                             safety_distance)
            self.logerr("Safe Speed: " + str(safe_speed))
            return safe_speed
        else:
            # If safety distance is reached, drive with same speed as
            # Object in front
            # TODO:
            # Incooperate overtaking ->
            # Communicate with decision tree about overtaking
            self.logerr("saftey distance good; Speed from obstacle: " +
                        str(self.obstacle[1]))
            return self.obstacle[1]

    def __get_current_velocity(self, data: CarlaSpeedometer):
        """_summary_

        Args:
            data (CarlaSpeedometer): _description_
        """
        self.__current_velocity = float(data.data)

    def __set_trajectory(self, data: Path):
        """Recieve trajectory from global planner

        Args:
            data (Path): Trajectory path
        """
        self.__trajectory = data

    def __set_speed_limits_opendrive(self, data: Float32MultiArray):
        """Recieve speed limits from OpenDrive via global planner

        Args:
            data (Float32MultiArray): speed limits per waypoint
        """
        self.__speed_limits_OD = data.data

    def __current_position_callback(self, data: PoseStamped):
        """Get current position and check if next waypoint is reached
            If yes -> update current waypoint and speed limit

        Args:
            data (PoseStamped): Current position from perception
        """
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
            self.speed_limit = \
                self.__speed_limits_OD[self.__current_wp_index]

    def run(self):
        """
        Control loop
        :return:
        """
        def loop(timer_event=None):
            if self.collision_ahead is False:
                self.velocity_pub.publish(self.speed_limit)
        self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    """
    main function starts the ACC node
    :param args:
    """
    roscomp.init('ACC')

    try:
        node = ACC()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

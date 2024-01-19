#!/usr/bin/env python
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
import time


class ACC(CompatibleNode):
    """
    This node recieves a possible collision and
    """

    def __init__(self):
        super(ACC, self).__init__('ACC')
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 1)
        self.current_speed = None  # m/ss

        # Get current speed
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
            self._set_distance,
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

        self.pose_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1)
        self.approx_speed_sub = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/cc_speed",
            self.__approx_speed_callback,
            qos_profile=1)
        # Publish desired speed to acting
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
        # Distance and speed from possible collsion object
        self.obstacle_speed: tuple = None
        # Obstalce distance
        self.obstacle_distance = None
        # Current speed limit
        self.speed_limit: float = None  # m/s

        self.logdebug("ACC initialized")

    def _set_distance(self, data: Float32):
        """Get min distance to object in front from perception

        Args:
            data (Float32): Minimum Distance from LIDAR
        """
        self.obstacle_distance = data.data

    def __approx_speed_callback(self, data: Float32):
        """Safe approximated speed form obstacle in front together with
        timestamp when recieved.
        Timestamp is needed to check wether we still have a vehicle in front

        Args:
            data (Float32): Speed from obstacle in front
        """
        # self.logerr("ACC: Approx speed recieved: " + str(data.data))
        self.obstacle_speed = (time.time(), data.data)

    def __get_current_velocity(self, data: CarlaSpeedometer):
        """_summary_

        Args:
            data (CarlaSpeedometer): _description_
        """
        self.__current_velocity = float(data.speed)

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
            """
            Checks if distance to a possible object is too small and
            publishes the desired speed to motion planning
            """
            if self.obstacle_speed is not None:
                # Check if too much time has passed since last speed update
                if self.obstacle_speed[0] + 0.5 < time.time():
                    self.obstacle_speed = None

            if self.obstacle_distance is not None and \
                    self.obstacle_speed is not None and \
                    self.__current_velocity is not None:
                # If we have obstalce speed and distance, we can
                # calculate the safe speed
                safety_distance = CollisionCheck.calculate_rule_of_thumb(
                    False, self.__current_velocity)
                if self.obstacle_distance < safety_distance:
                    # If safety distance is reached, we want to reduce the
                    # speed to meet the desired distance
                    safe_speed = self.obstacle_speed[1] * \
                        (self.obstacle_distance / safety_distance)
                    self.velocity_pub.publish(safe_speed)
                else:
                    # If safety distance is reached, drive with same speed as
                    # Object in front
                    self.velocity_pub.publish(self.obstacle_speed[1])

            elif self.speed_limit is not None:
                # If we have no obstacle, we want to drive with the current
                # speed limit
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

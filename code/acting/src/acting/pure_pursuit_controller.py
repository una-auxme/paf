#!/usr/bin/env python
import math
from math import atan, sin

import ros_compatibility as roscomp
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import Point, PoseStamped, Pose
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32
from acting.msg import Debug
import rospy
# import numpy as np

from helper_functions import vector_angle
from trajectory_interpolation import points_to_vector

MIN_LD_V: float = 3.0
LOOK_AHEAD_DIS = 3
MIN_L_A_DIS = 1
MAX_L_A_DIS = 15


class PurePursuitController(CompatibleNode):
    def __init__(self):
        super(PurePursuitController, self).__init__('pure_pursuit_controller')
        self.loginfo('PurePursuitController node started')

        self.control_loop_rate = self.get_param('control_loop_rate', 0.05)
        self.role_name = self.get_param('role_name', 'ego_vehicle')

        self.position_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory",
            self.__set_path,
            qos_profile=1)

        self.path_sub: Subscriber = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.__set_position,
            qos_profile=1)

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_velocity,
            qos_profile=1)

        self.heading_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__set_heading,
            qos_profile=1
        )

        self.pure_pursuit_steer_pub: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            qos_profile=1)

        self.pure_pursuit_steer_target_pub: Publisher = self.new_publisher(
            Pose,
            f"/paf/{self.role_name}/pure_pursuit_steer_target_wp",
            qos_profile=1)

        self.debug_publisher: Publisher = self.new_publisher(
            Debug,
            f"/paf/{self.role_name}/pure_p_debug",
            qos_profile=1)

        self.targetwp_publisher: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/current_target_wp",
            qos_profile=1)

        self.currentx_publisher: Publisher = self.new_publisher(
            Float32,
            f"/paf/{self.role_name}/current_x",
            qos_profile=1)

        self.__position: (float, float) = None  # x, y
        self.__last_pos: (float, float) = None
        self.__path: Path = None
        self.__heading: float = None
        self.__velocity: float = None
        self.__tp_idx: int = 0  # target waypoint index
        # error when there are no targets

        self.time_set = False
        self.checker = False
        self.checkpoint_time = -1

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo('PurePursuitController node running')

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__path is None:
                self.logdebug("PurePursuitController hasn't received a path "
                              "yet and can therefore not publish steering")
                return

            if self.__position is None:
                self.logdebug("PurePursuitController hasn't received the "
                              "position of the vehicle yet "
                              "and can therefore not publish steering")
                return

            if self.__heading is None:
                self.logdebug("PurePursuitController hasn't received the "
                              "heading of the vehicle yet and "
                              "can therefore not publish steering")
                return

            if self.__velocity is None:
                self.logdebug("PurePursuitController hasn't received the "
                              "velocity of the vehicle yet "
                              "and can therefore not publish steering")
                return

            self.pure_pursuit_steer_pub.publish(self.__calculate_steer())

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __calculate_steer(self) -> float:
        """
        Calculates the steering angle based on the current information
        :return:
        """
        l_vehicle = 2.85  # wheelbase
        k_ld = 0.5  # TODO: tune
        look_ahead_dist = LOOK_AHEAD_DIS  # offset so that ld is never zero

        if self.__velocity < 0:
            # backwards driving is not supported, TODO why check this here?
            return 0.0
        elif round(self.__velocity, 1) < MIN_LD_V:
            # Offset for low velocity state
            look_ahead_dist += 0.0  # no offset
        else:
            look_ahead_dist += k_ld * (self.__velocity - MIN_LD_V)

        # look_ahead_dist = np.clip(k_ld * self.__velocity,
        # MIN_L_A_DIS, MAX_L_A_DIS)

        # Get the target position on the trajectory in look_ahead distance
        self.__tp_idx = self.__get_target_point_index(look_ahead_dist)
        target_wp: PoseStamped = self.__path.poses[self.__tp_idx]
        # Get the vector from the current position to the target position
        target_v_x, target_v_y = points_to_vector((self.__position[0],
                                                   self.__position[1]),
                                                  (target_wp.pose.position.x,
                                                   target_wp.pose.position.y))
        # Get the target heading from that vector
        target_vector_heading = vector_angle(target_v_x, target_v_y)
        # Get the error between current heading and target heading
        alpha = target_vector_heading - self.__heading
        # Steering_angle is arctan (l_vehicle / R)
        # R is look_ahead_dist / 2 * sin(alpha)
        # https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html

        steering_angle = atan((2 * l_vehicle * sin(alpha)) / look_ahead_dist)

        # for debugging ->
        debug_msg = Debug()
        debug_msg.heading = self.__heading
        debug_msg.target_heading = target_vector_heading
        debug_msg.l_distance = look_ahead_dist
        debug_msg.steering_angle = steering_angle
        self.debug_publisher.publish(debug_msg)
        # <-
        self.pure_pursuit_steer_target_pub.publish(target_wp.pose)

        # not beautiful but works, eliminates the first
        # second because for some reason
        # the positional data from the GNSS is completely broken at the start
        if not self.time_set:
            self.checkpoint_time = rospy.get_time()
            self.time_set = True
        if not self.checker and (self.checkpoint_time < rospy.get_time() - 1):
            self.checker = True

        if self.checker:
            self.targetwp_publisher.publish((target_wp.pose.position.x-984.5))
            self.currentx_publisher.publish(self.__position[0]-984.5)

        return steering_angle

    def __set_position(self, data: PoseStamped, min_diff=0.001):
        """
        Updates the current position of the vehicle
        To avoid problems when the car is stationary, new positions will only
        be accepted, if they are a certain distance from the current one
        :param data: new position as PoseStamped
        :param min_diff: minium difference between new and current point for
        the new point to be accepted
        :return:
        """
        # No position yet: always get the published position
        if self.__position is None:
            x0 = data.pose.position.x
            y0 = data.pose.position.y
            self.__position = (x0, y0)
            return

        # check if the new position is valid
        dist = self.__dist_to(data.pose.position)
        if dist < min_diff:
            # if new position is to close to current, do not accept it
            # too close = closer than min_diff = 0.001 meters
            # for debugging purposes:
            self.logdebug("New position disregarded, "
                          f"as dist ({round(dist, 3)}) to current pos "
                          f"< min_diff ({round(min_diff, 3)})")
            return
        # TODO: why save the old position if it is never used again?
        old_x = self.__position[0]
        old_y = self.__position[1]
        self.__last_pos = (old_x, old_y)
        new_x = data.pose.position.x
        new_y = data.pose.position.y
        self.__position = (new_x, new_y)

    def __set_path(self, data: Path):
        path_len = len(data.poses)
        if path_len < 1:
            self.loginfo("Pure Pursuit: Empty path received and disregarded")
            return
        self.__path = data

    def __set_heading(self, data: Float32):
        """
        Updates the current heading
        :return:
        """
        self.__heading = data.data

    def __set_velocity(self, data: CarlaSpeedometer):
        self.__velocity = data.speed

    def __get_target_point_index(self, ld: float) -> int:
        """
        Get the index of the target point on the current trajectory based on
        the look ahead distance.
        :param ld: look ahead distance
        :return:
        """
        # if path has less than 2 poses, break
        if len(self.__path.poses) < 2:
            return -1

        # initialize min dist and idx very high and -1
        min_dist = 10e1000
        min_dist_idx = -1
        # might be more elegant to only look at points
        # _ahead_ of the closest point on the trajectory
        for i in range(self.__tp_idx, len(self.__path.poses)):
            pose: PoseStamped = self.__path.poses[i]
            dist = self.__dist_to(pose.pose.position)
            dist2ld = dist - ld
            # can be optimized
            if min_dist > dist2ld > 0:
                min_dist = dist2ld
                min_dist_idx = i
        return min_dist_idx

    def __dist_to(self, pos: Point) -> float:
        """
        Distance between current position and target position (only (x,y))
        :param pos: targeted position
        :return: distance
        """
        x_current = self.__position[0]
        y_current = self.__position[1]
        x_target = pos.x
        y_target = pos.y
        d = (x_target - x_current)**2 + (y_target - y_current)**2
        return math.sqrt(d)


def main(args=None):
    """
      main function starts the pure pursuit controller node
      :param args:
    """
    roscomp.init('pure_pursuit_controller', args=args)

    try:
        node = PurePursuitController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == '__main__':
    main()

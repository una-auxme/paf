#!/usr/bin/env python
import math
from math import atan, sin
import ros_compatibility as roscomp
import rospy
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32
from acting.msg import Debug
from visualization_msgs.msg import MarkerArray
import numpy as np
import shapely

import mapping_common.mask
import mapping_common.hero
from mapping_common.transform import Vector2, Point2
from mapping_common.markers import debug_marker, debug_marker_array

from typing import Optional

from control.cfg import PurePursuitConfig
from dynamic_reconfigure.server import Server


# Constant: wheelbase of car
L_VEHICLE = 2.85

MARKER_NAMESPACE = "pp_controller"
PP_CONTROLLER_MARKER_COLOR = (20 / 255, 232 / 255, 95 / 255, 0.5)


class PurePursuitController(CompatibleNode):
    def __init__(self):
        super(PurePursuitController, self).__init__("pure_pursuit_controller")
        self.loginfo("PurePursuitController node started")

        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.trajectory_sub: Subscriber = self.new_subscription(
            Path, "/paf/acting/trajectory_local", self.__set_trajectory, qos_profile=1
        )

        # self.position_sub: Subscriber = self.new_subscription(
        #     PoseStamped,
        #     "/paf/acting/current_pos",
        #     self.__set_position,
        #     qos_profile=1,
        # )

        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_velocity,
            qos_profile=1,
        )

        # self.heading_sub: Subscriber = self.new_subscription(
        #     Float32,
        #     "/paf/acting/current_heading",
        #     self.__set_heading,
        #     qos_profile=1,
        # )

        self.pure_pursuit_steer_pub: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/pure_pursuit_steer", qos_profile=1
        )

        self.debug_msg_pub: Publisher = self.new_publisher(
            Debug, f"/paf/{self.role_name}/pure_p_debug", qos_profile=1
        )

        # Publish debugging marker
        self.marker_publisher: Publisher = self.new_publisher(
            MarkerArray,
            f"/paf/{self.role_name}/control/pp_debug_markers",
            qos_profile=1,
        )

        # self.__position: Optional[tuple[float, float]] = None  # x, y
        self.__path: Optional[Path] = None
        # self.__heading: Optional[float] = None
        self.__velocity: Optional[float] = None

        # Tuneable Values for PurePursuit-Algorithm
        self.K_LAD: float
        self.MIN_LA_DISTANCE: float
        self.MAX_LA_DISTANCE: float
        self.K_PUB: float
        Server(PurePursuitConfig, self.dynamic_reconfigure_callback)

    def dynamic_reconfigure_callback(self, config: "PurePursuitConfig", level):
        self.K_LAD = config["k_lad"]
        self.MIN_LA_DISTANCE = config["min_la_distance"]
        self.MAX_LA_DISTANCE = config["max_la_distance"]
        self.K_PUB = -config["k_pub"]  # -0.80  # (-4.75) would be optimal in dev-launch
        # "-1" because it is inverted to the steering carla expects

        return config

    def run(self):
        """
        Starts the main loop of the node
        :return:
        """
        self.loginfo("PurePursuitController node running")

        def loop(timer_event=None):
            """
            Main loop of the acting node
            :param timer_event: Timer event from ROS
            :return:
            """
            if self.__path is None:
                rospy.logwarn_throttle(
                    1.0,
                    "PurePursuitController hasn't received a path "
                    "yet and can therefore not publish steering",
                )
                return

            # if self.__position is None:
            #     self.logdebug(
            #         "PurePursuitController hasn't received the "
            #         "position of the vehicle yet "
            #         "and can therefore not publish steering"
            #     )
            #     return

            # if self.__heading is None:
            #     self.logdebug(
            #         "PurePursuitController hasn't received the "
            #         "heading of the vehicle yet and "
            #         "can therefore not publish steering"
            #     )
            #     return

            if self.__velocity is None:
                rospy.logwarn_throttle(
                    1.0,
                    "PurePursuitController hasn't received the "
                    "velocity of the vehicle yet "
                    "and can therefore not publish steering",
                )
                return

            steering_angle = self.__calculate_steer()
            if steering_angle is None:
                rospy.logerr_throttle(
                    0.5, "PurePursuitController: Failed to calculate steering"
                )
            else:
                self.pure_pursuit_steer_pub.publish(steering_angle)

        self.new_timer(self.control_loop_rate, loop)
        self.spin()

    def __calculate_steer(self) -> Optional[float]:
        """
        Calculates the steering angle based on the current information
        :return:
        """
        hero = mapping_common.hero.create_hero_entity()
        hero_front_x = hero.get_front_x()
        front_point = Point2.new(hero_front_x, 0.0)
        front_point_s = front_point.to_shapely()
        trajectory_line = mapping_common.mask.ros_path_to_line(self.__path)
        # Calculate the distance on the traj to the front_point
        # (front_point is projected onto the traj)
        front_dist: float = trajectory_line.line_locate_point(other=front_point_s)
        (_, trajectory_line) = mapping_common.mask.split_line_at(
            trajectory_line, front_dist
        )
        if trajectory_line is None:
            return None
        # Prepend the front_point to the trajectory
        trajectory_line = shapely.LineString(
            np.append(
                [[front_point.x(), front_point.y()]],
                np.array(trajectory_line.coords),
                axis=0,
            )
        )

        # la_dist = MIN_LA_DISTANCE <= K_LAD * velocity <= MAX_LA_DISTANCE
        look_ahead_dist = np.clip(
            self.K_LAD * self.__velocity, self.MIN_LA_DISTANCE, self.MAX_LA_DISTANCE
        )
        # Get the target position on the trajectory in look_ahead distance
        (look_ahead_traj, _) = mapping_common.mask.split_line_at(
            trajectory_line, look_ahead_dist
        )
        if look_ahead_traj is None:
            return None
        # Last coordinate of look_ahead_traj is the point we look at
        (target_x, target_y) = look_ahead_traj.coords[-1]
        target_point = Point2.new(target_x, target_y)

        # Get the vector from the current position to the target position
        target_vector = front_point.vector_to(target_point)
        # Get the error between current heading and target heading
        alpha = Vector2.forward().angle_to(target_vector)
        # https://thomasfermi.github.io/Algorithms-for-Automated-Driving/Control/PurePursuit.html
        steering_angle = atan((2 * L_VEHICLE * sin(alpha)) / look_ahead_dist)
        steering_angle = self.K_PUB * steering_angle  # Needed for unknown reason

        # for debugging ->
        target_marker = debug_marker(target_point, color=PP_CONTROLLER_MARKER_COLOR)
        target_v_marker = debug_marker(
            (front_point, target_point), color=PP_CONTROLLER_MARKER_COLOR
        )
        text_offset = (front_point + (target_vector * 0.5)).vector()
        steer_angle_marker = debug_marker(
            f"PP angle: {math.degrees(steering_angle):4.1f} deg",
            offset=text_offset,
            color=(1.0, 1.0, 1.0, 1.0),
        )
        m_array = debug_marker_array(
            MARKER_NAMESPACE, [target_marker, target_v_marker, steer_angle_marker]
        )
        self.marker_publisher.publish(m_array)

        debug_msg = Debug()
        debug_msg.heading = 0.0
        debug_msg.target_heading = alpha
        debug_msg.l_distance = look_ahead_dist
        debug_msg.steering_angle = steering_angle
        self.debug_msg_pub.publish(debug_msg)
        # <-
        return steering_angle

    # def __get_target_point_index(self, ld: float) -> int:
    #     """
    #     Get the index of the target point on the current trajectory based on
    #     the look ahead distance.
    #     :param ld: look ahead distance
    #     :return:
    #     """
    #     if len(self.__path.poses) < 2:
    #         return -1

    #     closest_index = np.argmin(
    #         np.array([self.__dist_to(pose.pose.position) for pose in self.__path.poses])
    #     )

    #     ld_distances = np.array(
    #         [
    #             self.__dist_to(pose.pose.position) - ld
    #             for pose in self.__path.poses[closest_index:]
    #         ]
    #     )
    #     min_dist_idx = np.argmin(np.abs(ld_distances))
    #     return closest_index + min_dist_idx

    # def __is_ahead(self, pos: Tuple[float, float]) -> bool:
    #     x, y = pos
    #     c_x, c_y = self.__position
    #     to_car = np.array([x - c_x, y - c_y])
    #     heading = self.__rotate_vector_2d(np.array([1.0, 0.0]), self.__heading)

    #     return np.dot(to_car, heading) > 1

    # def __rotate_vector_2d(self, vector, angle_rad):
    #     rotation_matrix = np.array(
    #         [
    #             [np.cos(angle_rad), -np.sin(angle_rad)],
    #             [np.sin(angle_rad), np.cos(angle_rad)],
    #         ]
    #     )

    #     return rotation_matrix @ np.array(vector)

    # def __dist_to(self, pos: Point) -> float:
    #     """
    #     Distance between current position and target position (only (x,y))
    #     :param pos: targeted position
    #     :return: distance
    #     """
    #     x_current = self.__position[0]
    #     y_current = self.__position[1]
    #     x_target = pos.x
    #     y_target = pos.y
    #     d = (x_target - x_current) ** 2 + (y_target - y_current) ** 2
    #     return math.sqrt(d)

    # def __set_position(self, data: PoseStamped, min_diff=0.001):
    #     """
    #     Updates the current position of the vehicle
    #     To avoid problems when the car is stationary, new positions will only
    #     be accepted, if they are a certain distance from the current one
    #     :param data: new position as PoseStamped
    #     :param min_diff: minium difference between new and current point for
    #     the new point to be accepted
    #     :return:
    #     """
    #     # No position yet: always get the published position
    #     if self.__position is None:
    #         x0 = data.pose.position.x
    #         y0 = data.pose.position.y
    #         self.__position = (x0, y0)
    #         return
    #     # check if the new position is valid
    #     dist = self.__dist_to(data.pose.position)
    #     if dist < min_diff:
    #         # if new position is to close to current, do not accept it
    #         # too close = closer than min_diff = 0.001 meters
    #         # for debugging purposes:
    #         self.logdebug(
    #             "New position disregarded, "
    #             f"as dist ({round(dist, 3)}) to current pos "
    #             f"< min_diff ({round(min_diff, 3)})"
    #         )
    #         return
    #     new_x = data.pose.position.x
    #     new_y = data.pose.position.y
    #     self.__position = (new_x, new_y)

    def __set_trajectory(self, data: Path):
        path_len = len(data.poses)
        if path_len < 1:
            self.loginfo("Pure Pursuit: Empty path received and disregarded")
            return
        self.__path = data

    # def __set_heading(self, data: Float32):
    #     self.__heading = data.data

    def __set_velocity(self, data: CarlaSpeedometer):
        self.__velocity = data.speed


def main(args=None):
    """
    main function starts the pure pursuit controller node
    :param args:
    """
    roscomp.init("pure_pursuit_controller", args=args)

    try:
        node = PurePursuitController()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

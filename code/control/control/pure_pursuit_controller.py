from typing import Optional
import math
from math import atan, sin
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription

from carla_msgs.msg import CarlaSpeedometer
from nav_msgs.msg import Path
from std_msgs.msg import Float32
from visualization_msgs.msg import MarkerArray


import mapping_common.mask
import mapping_common.hero
from mapping_common.transform import Vector2, Point2
from mapping_common.markers import debug_marker, debug_marker_array


# Constant: wheelbase of car
L_VEHICLE = 2.85

MARKER_NAMESPACE = "pp_controller"
PP_CONTROLLER_MARKER_COLOR = (20 / 255, 232 / 255, 95 / 255, 0.5)


class PurePursuitController(Node):
    def __init__(self):
        super().__init__("pure_pursuit_controller")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Configuration parameters
        self.control_loop_rate = (
            self.declare_parameter("control_loop_rate", 0.05)
            .get_parameter_value()
            .double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )

        self.trajectory_sub: Subscription = self.new_subscription(
            Path, "/paf/acting/trajectory_local", self.__set_trajectory, qos_profile=1
        )

        self.velocity_sub: Subscription = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__set_velocity,
            qos_profile=1,
        )

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

        self.__path: Optional[Path] = None
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

        def loop_handler(timer_event=None):
            try:
                loop()
            except Exception as e:
                rospy.logfatal(e)

        self.new_timer(self.control_loop_rate, loop_handler)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def __calculate_steer(self) -> Optional[float]:
        """
        Calculates the steering angle based on the current information
        :return:
        """
        hero = mapping_common.hero.create_hero_entity()
        hero_front_x = hero.get_front_x()
        front_point = Point2.new(hero_front_x, 0.0)
        trajectory_line = mapping_common.mask.build_trajectory_from_start(
            self.__path, front_point, max_centering_dist=None
        )
        if trajectory_line is None:
            return None

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
        # Limit alpha to +- pi/2 to avoid having a reduced steering angle
        # when alpha is too big
        alpha = min(max(-math.pi * 0.5, alpha), math.pi * 0.5)
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

    def __set_trajectory(self, data: Path):
        path_len = len(data.poses)
        if path_len < 1:
            self.loginfo("Pure Pursuit: Empty path received and disregarded")
            return
        self.__path = data

    def __set_velocity(self, data: CarlaSpeedometer):
        self.__velocity = data.speed


def main(args=None):
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

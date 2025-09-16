"""
This node publishes the distance to the next waypoint and the
stop line of an intersection specified by the global waypoint in front of
it.
"""

from typing import Optional, List

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaRoute
from nav_msgs.msg import Path

from perception_interfaces.msg import Waypoint

import math


class GlobalPlanDistance(Node):

    def __init__(self):
        super().__init__("global_plan_distance_publisher")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # basic info
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )

        self.current_pos = None
        self.trajectory_local = None
        self.global_route = None
        self.road_options: Optional[List[int]] = None

        # Subscriber
        self.pos_subscriber = self.create_subscription(
            PoseStamped,
            "/paf/" + self.role_name + "/current_pos",
            self.update_position,
            qos_profile=1,
        )

        # Get trajectory only for checking of the motion_planning has initialized
        self.trajectory_local_sub: Subscription = self.create_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory_local",
            self.__set_trajectory_local,
            qos_profile=1,
        )

        self.global_plan_subscriber = self.create_subscription(
            CarlaRoute,
            "/carla/" + self.role_name + "/global_plan",
            self.update_global_route,
            qos_profile=1,
        )

        self.waypoint_publisher: Publisher = self.create_publisher(
            Waypoint, "/paf/" + self.role_name + "/current_waypoint", qos_profile=1
        )

        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def __set_trajectory_local(self, data: Path):
        """Receive trajectory from motion planner

        Args:
            data (Path): Trajectory path
        """
        self.trajectory_local = data

    def update_position(self, pos):
        """
        Updates the current position based on the most upto date
        IMU, Speedometer and GNSS sensor data.
        :return:
        """

        def distance(a, b):
            d_x = (a.x - b.x) ** 2
            d_y = (a.y - b.y) ** 2
            return math.sqrt(d_x + d_y)

        self.current_pos = pos.pose

        # check if the global route has been published and that there are still
        # points to navigate to
        if (
            self.global_route is None
            or len(self.global_route) == 0
            or self.road_options is None
            or len(self.road_options) == 0
            or self.trajectory_local is None
        ):
            return

        current_distance = distance(
            self.global_route[0].position, self.current_pos.position
        )
        next_distance = distance(
            self.global_route[1].position, self.current_pos.position
        )

        waypoint_type = Waypoint.TYPE_UNKNOWN
        # if the road option indicates an intersection, the distance to the
        # next waypoint is also the distance to the stop line
        if self.road_options[0] in {
            CarlaRoute.VOID,
            CarlaRoute.LEFT,
            CarlaRoute.RIGHT,
            CarlaRoute.STRAIGHT,
        }:
            waypoint_type = Waypoint.TYPE_INTERSECTION
        elif self.road_options[0] in {
            CarlaRoute.CHANGELANELEFT,
            CarlaRoute.CHANGELANERIGHT,
        }:
            waypoint_type = Waypoint.TYPE_LANECHANGE

        current_waypoint = Waypoint(
            waypoint_type=waypoint_type,
            distance=current_distance,
            roadOption=self.road_options[0],
            position=self.global_route[0].position,
        )
        self.waypoint_publisher.publish(current_waypoint)

        # if we reached the next waypoint, pop it and the next point will
        # be published
        if current_distance < 2.5 or next_distance < current_distance:
            self.road_options.pop(0)
            self.global_route.pop(0)

            if (
                len(self.road_options) > 1
                and self.road_options[0]
                in {CarlaRoute.CHANGELANELEFT, CarlaRoute.CHANGELANERIGHT}
                and self.road_options[0] == self.road_options[1]
            ):
                self.road_options[1] = CarlaRoute.LANEFOLLOW

    def update_global_route(self, route):
        """
        Callback if the global route is published, saves the route and road
        options locally
        :param: route, CarlaRoute holding global route
        :return:
        """
        if self.global_route is None:
            self.global_route = list(route.poses)
            self.road_options = list(route.road_options)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = GlobalPlanDistance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

"""
This node publishes the distance to the next waypoint and the
stop line of an intersection specified by the global waypoint in front of
it.
"""

from typing import Optional, List

import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped
from carla_msgs.msg import CarlaRoute
from nav_msgs.msg import Path
from perception_interfaces.msg import Waypoint
from planning_interfaces.srv import GetCarlaRoute

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

        # Subscriptions
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

        self.create_subscription(
            msg_type=Bool,
            topic=f"/paf/{self.role_name}/data/planning/global_plan_updated",
            callback=self.update_global_route,
            qos_profile=10,
        )

        # Publishers
        self.waypoint_publisher: Publisher = self.create_publisher(
            Waypoint, "/paf/" + self.role_name + "/current_waypoint", qos_profile=1
        )

        # Service clients
        self.client_callback_group = (
            rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        self.global_plan_client = self.create_client(
            GetCarlaRoute,
            f"/paf/{self.role_name}/data/planning/get_global_plan",
            callback_group=self.client_callback_group,
        )
        for client in self.clients:
            while not client.wait_for_service(timeout_sec=2.0):
                self.get_logger().warn(
                    f"Waiting for the {client.service_name} service "
                    "to become available..."
                )

        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def __set_trajectory_local(self, data: Path):
        """Receive trajectory from motion planner

        Args:
            data (Path): Trajectory path
        """
        self.trajectory_local = data

    async def update_position(self, pos):
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
        if self.global_route is None:
            await self.update_global_route()
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
            road_option=self.road_options[0],
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

    async def update_global_route(self, data: Bool = Bool(data=True)):
        """
        Callback if the global route is published, saves the route and road
        options locally
        :param: route, CarlaRoute holding global route
        :return:
        """
        if not data.data:
            return
        if self.global_route is not None:
            return

        self.get_logger().info("Requesting global plan...")
        req = GetCarlaRoute.Request()
        response: Optional[GetCarlaRoute.Response] = (
            await self.global_plan_client.call_async(req)
        )
        if response is None:
            self.get_logger().warn(
                f"{self.global_plan_client.service_name} service returned None."
            )
            return
        if not response.success:
            self.get_logger().warn(
                f"{self.global_plan_client.service_name} service failed: "
                f"{response.msg}."
            )
            return
        route: CarlaRoute = response.data
        self.global_route = list(route.poses)
        self.road_options = list(route.road_options)
        self.get_logger().info("Global plan received successfully.")


def main(args=None):
    rclpy.init(args=args)

    try:
        node = GlobalPlanDistance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

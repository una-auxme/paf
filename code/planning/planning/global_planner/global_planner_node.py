from collections import deque
from typing import Deque, Optional
from xml.etree import ElementTree as eTree

import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.service import Service

from transforms3d.euler import euler2quat
from carla_msgs.msg import CarlaRoute
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from std_msgs.msg import Float32MultiArray, Bool
from planning_interfaces.srv import (
    GetCarlaRoute,
    GetOpenDriveString,
    GetPath,
    GetSpeedLimits,
)

from mapping_common.transform import Point2

from paf_common.exceptions import emsg_with_trace

from .preplanning_trajectory import OpenDriveConverter

# TODO: These definition do not align with the CarlaRoute.RIGHT, etc.. definitions.
# -> Check for possible bugs
RIGHT = 1
LEFT = 2
FORWARD = 3


class PrePlanner(Node):
    """
    This node is responsible for collecting all data needed for the
    preplanning and calculating a trajectory based on the OpenDriveConverter
    from preplanning_trajectory.py.
    Subscribed/needed topics:
    - OpenDrive Map:          /carla/{role_name}/OpenDRIVE
                 or:          /carla/world_info
    - global Plan:            /carla/{role_name}/global_plan
    - current agent position: /paf/{role_name}/current_pos
    Published topics:
    - preplanned trajectory:  /paf/{role_name}/trajectory_global
    - prevailing speed limits:/paf/{role_name}/speed_limits_OpenDrive
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Working variables
        self.odc = None
        self.route_recalculation_required: bool = True
        self.position_stabilized: bool = False
        self.last_agent_positions: Deque[Point] = deque()
        self.last_agent_positions_count_target = 5
        self.agent_pos = None
        self.agent_ori = None

        # Result/service variables
        self.global_trajectory: Optional[Path] = None
        self.speed_limits: Optional[Float32MultiArray] = None

        # Parameters
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )
        self.distance_spawn_to_first_wp = (
            self.declare_parameter(
                "distance_spawn_to_first_wp",
                100.0,
            )
            .get_parameter_value()
            .double_value
        )

        # Services
        # Get created only after data is available
        self.speed_limits_service: Optional[Service] = None
        self.global_trajectory_service: Optional[Service] = None

        # Subscriptions
        self.create_subscription(
            msg_type=Bool,
            topic=f"/paf/{self.role_name}/data/planning/open_drive_updated",
            callback=self.trigger_route_recalculation,
            qos_profile=10,
        )
        self.create_subscription(
            msg_type=Bool,
            topic=f"/paf/{self.role_name}/data/planning/global_plan_updated",
            callback=self.trigger_route_recalculation,
            qos_profile=10,
        )

        self.create_subscription(
            msg_type=PoseStamped,
            topic="/paf/" + self.role_name + "/current_pos",
            callback=self.position_callback,
            qos_profile=1,
        )

        # Publishers
        self.global_trajectory_updated_pub = self.create_publisher(
            msg_type=Bool,
            topic=f"/paf/{self.role_name}/data/planning/global_trajectory_updated",
            qos_profile=1,
        )

        self.speed_limit_updated_pub = self.create_publisher(
            msg_type=Bool,
            topic=f"/paf/{self.role_name}/data/planning/speed_limits_updated",
            qos_profile=1,
        )

        # Service clients
        self.client_callback_group = (
            rclpy.callback_groups.MutuallyExclusiveCallbackGroup()
        )
        self.open_drive_client = self.create_client(
            GetOpenDriveString,
            f"/paf/{self.role_name}/data/planning/get_open_drive",
            callback_group=self.client_callback_group,
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

    def trigger_route_recalculation(self, b: Bool = Bool(data=True)):
        if b.data:
            self.get_logger().info("Global route recalculation was triggered.")
            self.route_recalculation_required = True

    def update_global_trajectory(self, new_global_trajectory: Path):
        self.get_logger().info("Updated global trajectory.")
        self.global_trajectory = new_global_trajectory
        if self.global_trajectory_service is None:
            self.global_trajectory_service = self.create_service(
                GetPath,
                f"/paf/{self.role_name}/data/planning/get_global_trajectory",
                self.get_global_trajectory_service,
            )
            self.get_logger().info(
                f"Started {self.global_trajectory_service.service_name} service."
            )
        self.global_trajectory_updated_pub.publish(Bool(data=True))

    def get_global_trajectory_service(
        self, req: GetPath.Request, res: GetPath.Response
    ) -> GetPath.Response:
        if self.global_trajectory is not None:
            res.success = True
            res.msg = "success"
            res.data = self.global_trajectory
        else:
            res.success = False
            res.msg = "data not available"
        return res

    def update_speed_limits(self, new_speed_limits: Float32MultiArray):
        self.get_logger().info("Updated speed limits.")
        self.speed_limits = new_speed_limits
        if self.speed_limits_service is None:
            self.speed_limits_service = self.create_service(
                GetSpeedLimits,
                f"/paf/{self.role_name}/data/planning/get_speed_limits",
                self.get_speed_limits_service,
            )
            self.get_logger().info(
                f"Started {self.speed_limits_service.service_name} service."
            )
        self.speed_limit_updated_pub.publish(Bool(data=True))

    def get_speed_limits_service(
        self, req: GetSpeedLimits.Request, res: GetSpeedLimits.Response
    ) -> GetSpeedLimits.Response:
        if self.speed_limits is not None:
            res.success = True
            res.msg = "success"
            res.data = self.speed_limits
        else:
            res.success = False
            res.msg = "data not available"
        return res

    async def process_global_plan(self) -> bool:
        """
        when the global route gets updated a new trajectory is calculated with
        the help of OpenDriveConverter and published into
        '/paf/ self.role_name /trajectory_global'
        :param data: global Route
        """
        if self.odc is None:
            self.get_logger().warn("OpenDriveConverter not initialized yet.")
            return False

        if self.agent_pos is None or self.agent_ori is None:
            self.get_logger().warn("Agent pose not available yet.")
            return False

        req = GetCarlaRoute.Request()
        response: Optional[GetCarlaRoute.Response] = (
            await self.global_plan_client.call_async(req)
        )
        if response is None:
            self.get_logger().warn(
                f"{self.global_plan_client.service_name} service returned None."
            )
            return False
        if not response.success:
            self.get_logger().warn(
                f"{self.global_plan_client.service_name} service failed: "
                f"{response.msg}."
            )
            return False
        data: CarlaRoute = response.data
        self.get_logger().info("Processing global plan...")

        x_start = self.agent_pos.x  # 983.5
        y_start = self.agent_pos.y  # -5433.2
        x_target = data.poses[0].position.x
        y_target = data.poses[0].position.y
        if (
            abs(x_start - x_target) > self.distance_spawn_to_first_wp
            or abs(y_start - y_target) > self.distance_spawn_to_first_wp
        ):
            self.get_logger().warn(
                "Current agent-pose does not match the given global route"
            )
            return False

        # get the first turn command (1, 2, or 3)
        x_turn = None
        y_turn = None
        ind = 0
        for i, opt in enumerate(data.road_options):
            if opt == LEFT or opt == RIGHT or opt == FORWARD:
                x_turn = data.poses[i].position.x
                y_turn = data.poses[i].position.y
                ind = i
                break
        if x_turn is None or y_turn is None:
            self.get_logger().warn("Did not find first turn command")
            return False
        # if first target point is turning point
        if x_target == x_turn and y_target == y_turn:
            x_target = None
            y_target = None

        x_turn_follow = data.poses[ind + 1].position.x
        y_turn_follow = data.poses[ind + 1].position.y

        # Trajectory for the starting road segment
        self.odc.initial_road_trajectory(
            x_start,
            y_start,
            x_turn,
            y_turn,
            x_turn_follow,
            y_turn_follow,
            x_target,
            y_target,
            0,
            data.road_options[0],
        )

        n = len(data.poses)
        # iterating through global route to create trajectory
        for i in range(1, n - 1):
            # self.loginfo(f"Preplanner going throug global plan {i+1}/{n}")

            x_target = data.poses[i].position.x
            y_target = data.poses[i].position.y
            action = data.road_options[i]

            x_target_next = data.poses[i + 1].position.x
            y_target_next = data.poses[i + 1].position.y
            self.odc.target_road_trajectory(
                x_target, y_target, x_target_next, y_target_next, action
            )

        self.odc.target_road_trajectory(
            data.poses[n - 1].position.x,
            data.poses[n - 1].position.y,
            None,
            None,
            data.road_options[n - 1],
        )
        # trajectory is now stored in the waypoints
        # waypoints = self.odc.waypoints
        waypoints = self.odc.remove_outliner(self.odc.waypoints)
        way_x = waypoints[0]
        way_y = waypoints[1]
        way_yaw = waypoints[2]
        speed_limits = Float32MultiArray(data=waypoints[3])
        self.update_speed_limits(speed_limits)

        # Transforming the calculated waypoints into a Path msg
        stamped_poses = []
        for i in range(len(way_x)):
            position = Point(x=way_x[i], y=way_y[i], z=0.0)  # way_speed[i])
            quaternion = euler2quat(0.0, 0.0, way_yaw[i])
            orientation = Quaternion(
                x=quaternion[1], y=quaternion[2], z=quaternion[3], w=quaternion[0]
            )
            pose = Pose(position=position, orientation=orientation)
            pos = PoseStamped()
            pos.header.stamp = self.get_clock().now().to_msg()
            pos.header.frame_id = "global"
            pos.pose = pose
            stamped_poses.append(pos)

        global_trajectory = Path()
        global_trajectory.header.stamp = self.get_clock().now().to_msg()
        global_trajectory.header.frame_id = "global"
        global_trajectory.poses = stamped_poses
        self.update_global_trajectory(global_trajectory)

        self.get_logger().info("Successfully processed global plan.")
        return True

    async def process_open_drive_data(self) -> bool:
        """
        when the map gets updated a new OpenDriveConverter instance is created
        (needed for the trajectory preplanning)
        :param opendrive: updated CarlaWorldInformation
        """
        req = GetOpenDriveString.Request()
        response: Optional[GetOpenDriveString.Response] = (
            await self.open_drive_client.call_async(req)
        )
        if response is None:
            self.get_logger().warn(
                f"{self.open_drive_client.service_name} service returned None."
            )
            return False
        if not response.success:
            self.get_logger().warn(
                f"{self.open_drive_client.service_name} service failed: {response.msg}."
            )
            return False
        opendrive: str = response.data
        self.get_logger().info("Processing open drive data...")

        root = eTree.fromstring(opendrive)

        roads = root.findall("road")
        road_ids = [int(road.get("id")) for road in roads]
        junctions = root.findall("junction")
        junction_ids = [int(junction.get("id")) for junction in junctions]

        odc = OpenDriveConverter(
            roads=roads,
            road_ids=road_ids,
            junctions=junctions,
            junction_ids=junction_ids,
        )

        odc.convert_roads()
        odc.convert_junctions()
        odc.filter_geometry()

        self.odc = odc

        self.get_logger().info("Successfully processed open drive data.")
        return True

    async def position_callback(self, data: PoseStamped):
        """
        when the position gets updated it gets stored into self.agent_pos and
        self.agent_ori
        (needed for the trajectory preplanning)
        :param data: updated CarlaWorldInformation
        """
        if len(self.last_agent_positions) < self.last_agent_positions_count_target:
            self.get_logger().info(
                "Waiting for agent positions", throttle_duration_sec=2
            )
            self.last_agent_positions.append(data.pose.position)
            self.agent_pos = None
            self.agent_ori = None
            return

        agent_pos = data.pose.position
        agent_point = Point2.new(agent_pos.x, agent_pos.y)

        # Check if our position has stabilized
        if not self.position_stabilized:
            self.position_stabilized = True
            for pos in self.last_agent_positions:
                pos_point = Point2.new(pos.x, pos.y)
                if pos_point.distance_to(agent_point) > 0.5:
                    self.position_stabilized = False

        self.last_agent_positions.popleft()
        self.last_agent_positions.append(agent_pos)

        if not self.position_stabilized:
            self.get_logger().info(
                "Waiting for agent position to stabilize",
                throttle_duration_sec=2,
            )
            self.agent_pos = None
            self.agent_ori = None
            return

        self.agent_pos = agent_pos
        self.agent_ori = data.pose.orientation

        if self.route_recalculation_required:
            self.get_logger().info(
                "Received a pose update -> recalculating route...",
                throttle_duration_sec=2,
            )
            try:
                if not await self.process_open_drive_data():
                    self.get_logger().error("Failed to process open drive data.")
                    return
                if not await self.process_global_plan():
                    self.get_logger().error("Failed to process the global plan.")
                    return
                self.route_recalculation_required = False
            except Exception as e:
                self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = PrePlanner()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

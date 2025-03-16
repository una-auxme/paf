#!/usr/bin/env python
# import tf.transformations
import math
import os
import sys
from typing import List

import numpy as np
import ros_compatibility as roscomp
import rospy
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, Point32
from nav_msgs.msg import Path
from perception.msg import LaneChange, Waypoint
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber, ServiceProxy
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16, String, Empty
from scipy.spatial.transform import Rotation as R

from utils import (
    NUM_WAYPOINTS,
    NUM_WAYPOINTS_OVERTAKE,
    NUM_WAYPOINTS_OVERTAKE_UNSTUCK,
    TARGET_DISTANCE_TO_STOP,
    convert_to_ms,
    spawn_car,
)


sys.path.append(os.path.abspath(sys.path[0] + "/../../planning/src/behavior_agent"))
from behaviors import behavior_speed as bs  # type: ignore # noqa: E402

from teb_planner_pa_msgs.srv import Plan, PlanRequest, PlanResponse
from mapping.msg import Map as MapMsg
from mapping_common.entity import Entity
from mapping_common.map import Map
from mapping_common.transform import Transform2D
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from copy import deepcopy
import time

# from scipy.spatial._kdtree import KDTree

from typing import Optional
from numpy.typing import NDArray

from dynamic_reconfigure.server import Server
from planning.cfg import MotionPlanConfig


UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE = 7.0


class MotionPlanning(CompatibleNode):
    """
    This node selects speeds based on the current behaviour and ACC to forward to the
    acting components. It also handles the generation of trajectories for overtaking
    maneuvers.
    """

    def __init__(self):
        super(MotionPlanning, self).__init__("MotionPlanning")
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)

        self.map: Optional[Map] = None
        # TODO: add type hints
        self.target_speed = 0.0
        self.target_velocity_selector = "not selected"
        self.__curr_behavior = None
        self.__acc_speed = 0.0
        self.__stopline = None  # (Distance, isStopline)
        self.__change_point = None  # (Distance, isLaneChange, roadOption)
        self.__collision_point = None
        # TODO: clarify what the overtake_status values mean (by using an enum or ...)
        self.__overtake_status = -1
        self.published = False
        self.current_pos: Optional[NDArray] = None
        self.current_heading: Optional[float] = None
        self.trajectory: Optional[Path] = None
        self.original_trajectory: Optional[Path] = None
        self.overtaking = False
        self.current_wp_idx: Optional[int] = None
        self.closest_idx: Optional[int] = None
        self.distance_to_trajectory: Optional[float] = None
        self.enhanced_path = None
        self.current_speed = None
        self.speed_limit = None
        self.__corners = None
        self.__in_corner = False
        self.calculated = False
        self.traffic_light_y_distance = np.inf
        # unstuck routine variables
        self.unstuck_distance = None
        self.unstuck_overtake_flag = False
        self.init_overtake_pos = None
        # Subscriber
        self.test_sub = self.new_subscription(
            Float32, f"/paf/{self.role_name}/spawn_car", spawn_car, qos_profile=1
        )
        self.speed_limit_sub = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/speed_limit",
            self.__set_speed_limit,
            qos_profile=1,
        )
        self.velocity_sub: Subscriber = self.new_subscription(
            CarlaSpeedometer,
            f"/carla/{self.role_name}/Speed",
            self.__get_current_velocity,
            qos_profile=1,
        )
        self.head_sub = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__set_heading,
            qos_profile=1,
        )

        self.trajectory_sub = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory_global",
            self.__set_trajectory,
            qos_profile=1,
        )

        self.route_update_sub = self.new_subscription(
            msg_type=Empty,
            topic=f"/paf/{self.role_name}/route_update",
            callback=self.__route_update_callback,
            qos_profile=1,
        )

        self.current_pos_sub = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.__set_current_pos,
            qos_profile=1,
        )
        self.curr_behavior_sub: Subscriber = self.new_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1,
        )
        self.emergency_sub: Subscriber = self.new_subscription(
            Bool,
            f"/paf/{self.role_name}/unchecked_emergency",
            self.__check_emergency,
            qos_profile=1,
        )
        self.acc_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/acc_velocity",
            self.__set_acc_speed,
            qos_profile=1,
        )

        self.stopline_sub: Subscriber = self.new_subscription(
            Waypoint,
            f"/paf/{self.role_name}/waypoint_distance",
            self.__set_stopline,
            qos_profile=1,
        )

        self.change_point_sub: Subscriber = self.new_subscription(
            LaneChange,
            f"/paf/{self.role_name}/lane_change_distance",
            self.__set_change_point,
            qos_profile=1,
        )

        self.coll_point_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/collision",
            self.__set_collision_point,
            qos_profile=1,
        )

        self.traffic_y_sub: Subscriber = self.new_subscription(
            Int16,
            f"/paf/{self.role_name}/Center/traffic_light_y_distance",
            self.__set_traffic_y_distance,
            qos_profile=1,
        )
        self.unstuck_distance_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/unstuck_distance",
            self.__set_unstuck_distance,
            qos_profile=1,
        )

        self.new_subscription(
            topic=self.get_param("~map_topic", "/paf/hero/mapping/init_data"),
            msg_type=MapMsg,
            callback=self.__map_callback,
            qos_profile=1,
        )

        # Publisher

        self.traj_pub: Publisher = self.new_publisher(
            msg_type=Path, topic=f"/paf/{self.role_name}/trajectory", qos_profile=1
        )
        self.velocity_pub: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/target_velocity", qos_profile=1
        )

        self.velocity_selector_pub: Publisher = self.new_publisher(
            String, f"/paf/{self.role_name}/target_velocity_selector", qos_profile=1
        )

        # TODO move up to subscribers
        self.wp_subs = self.new_subscription(
            Float32, f"/paf/{self.role_name}/current_wp", self.__set_wp, qos_profile=1
        )

        self.overtake_success_pub = self.new_publisher(
            Float32, f"/paf/{self.role_name}/overtake_success", qos_profile=1
        )

        # Service clients
        self.plan_service: ServiceProxy = self.new_client(
            srv_type=Plan, srv_name="/teb_planner_node_pa/plan"
        )
        self.__plan_request: PlanRequest = PlanRequest()

        # Debug
        self.debug_traj_pub: Publisher = self.new_publisher(
            msg_type=Path, topic="debug/traj", qos_profile=1
        )
        self.old_plan: Optional[Path] = None

        self.logdebug("MotionPlanning started")
        self.counter = 0

        reconfigure_server = Server(MotionPlanConfig, self.dynamic_reconfigure_callback)
        self.LOOKAHEAD_IDX_COUNT: int
        self.LOOKBEHIND_IDX_COUNT: int
        self.SELF_AS_START: bool

    def dynamic_reconfigure_callback(self, config: MotionPlanConfig, level):
        self.LOOKAHEAD_IDX_COUNT = config["lookahead_idx_count"]
        self.LOOKBEHIND_IDX_COUNT = config["lookbehind_idx_count"]
        self.SELF_AS_START = config["self_as_start"]
        return config

    def __set_unstuck_distance(self, data: Float32):
        """Set unstuck distance

        Args:
            data (Float32): Unstuck distance
        """
        self.unstuck_distance = data.data

    def __set_speed_limit(self, data: Float32):
        """Set current speed limit

        Args:
            data (Float32): Current speed limit
        """
        self.speed_limit = data.data

    def __get_current_velocity(self, data: CarlaSpeedometer):
        """Get current velocity from CarlaSpeedometer

        Args:
            data (CarlaSpeedometer): Current velocity
        """
        self.current_speed = float(data.speed)

    def __set_wp(self, data: Float32):
        """Recieve current waypoint index from ACC

        Args:
            data (Float32): Waypoint index
        """
        self.current_wp_idx = int(data.data)

    def __set_heading(self, data: Float32):
        """Set current Heading

        Args:
            data (Float32): Current Heading vom Subscriber
        """
        self.current_heading = data.data

    def __set_current_pos(self, data: PoseStamped):
        """set current position
        Args:
            data (PoseStamped): current position
        """
        self.current_pos = np.array(
            [data.pose.position.x, data.pose.position.y, data.pose.position.z]
        )

    def __set_traffic_y_distance(self, data):
        if data is not None:
            self.traffic_light_y_distance = data.data

    def __map_callback(self, map_msg: MapMsg):
        self.map = Map.from_ros_msg(map_msg)

    def change_trajectory(self, distance_obj):
        """update trajectory for overtaking and convert it
        to a new Path message

        Args:
            distance_obj (float): distance to overtake object
        """
        pose_list = self.trajectory.poses

        # Only use fallback
        self.generate_overtake_trajectory(distance_obj, pose_list)
        self.__overtake_status = 1
        self.overtake_success_pub.publish(self.__overtake_status)
        return

    def __direction_vector_to_quaternion(
        self, direction, reference=np.array([0, 0, 1])
    ):
        # Normalize the input direction vector
        direction = np.array(direction)
        direction_norm = np.linalg.norm(direction)
        if direction_norm == 0:
            raise ValueError("Direction vector cannot be zero.")
        direction = direction / direction_norm

        # Check if the direction is the same as the reference
        if np.allclose(direction, reference):
            return np.array([0, 0, 0, 1])  # No rotation needed, identity quaternion

        # Compute rotation axis and angle
        rotation_axis = np.cross(reference, direction)
        rotation_axis_norm = np.linalg.norm(rotation_axis)
        if rotation_axis_norm == 0:  # Vectors are collinear
            return (
                np.array([0, 0, 0, 1])
                if np.dot(reference, direction) > 0
                else np.array([1, 0, 0, 0])
            )

        rotation_axis = rotation_axis / rotation_axis_norm
        angle = np.arccos(np.clip(np.dot(reference, direction), -1.0, 1.0))

        # Create the quaternion
        quaternion = R.from_rotvec(rotation_axis * angle).as_quat()  # x, y, z, w format
        return quaternion

    def _generate_overtake_trajectory(
        self, distance_obj: Optional[float], unstuck: bool = False
    ) -> Optional[Path]:
        """Update the trajectory because we need an overtake behaviour.

        This modifies self.trajectory.
        Args:
            distance_obj (Optional[float]): _description_
            unstuck (bool, optional): _description_. Defaults to False.
        """
        if (
            self.closest_idx is None
            or self.trajectory is None
            or self.trajectory.poses is None
            or self.map is None
            or self.current_pos is None
            or self.current_heading is None
        ):
            return None
        start_time = time.time()
        req = self.__plan_request
        goal_index = (
            self.closest_idx + self.LOOKAHEAD_IDX_COUNT
        )  # Different for unstuck??
        behind_idx = max(1, self.closest_idx - self.LOOKBEHIND_IDX_COUNT)

        """Trying to look behind"""
        current: PoseStamped = PoseStamped()
        self.__set_orientation_from_heading(
            current.pose.orientation, self.current_heading
        )
        (current.pose.position.x, current.pose.position.y, current.pose.position.z) = (
            self.current_pos
        )

        if self.SELF_AS_START:
            req.request.start = current.pose
            req.request.waypoints = Path()
            import random

            # for curves this s very good.
            for _ in range(10):
                idx = random.randint(self.closest_idx + 1, goal_index)
                pose: PoseStamped = deepcopy(self.original_trajectory.poses[idx])
                self.__set_orientation_from_poses(
                    pose.pose.orientation,
                    self.original_trajectory.poses[idx],
                    self.original_trajectory.poses[idx - 1],
                )
                req.request.waypoints.poses.append(pose)
        else:
            req.request.waypoints = Path()
            req.request.waypoints.poses.append(current)

            req.request.start = self.original_trajectory.poses[behind_idx].pose
            self.__set_orientation_from_poses(
                req.request.start.orientation,
                self.original_trajectory.poses[behind_idx],
                self.original_trajectory.poses[behind_idx - 1],
            )
            # TODO: Fixed overtake Waypoint number... improve this
            """The distance to the object ahead should maybe also be taken into account.
                The original code however used it as an index which is totally incorrect.
            """

        req.request.goal = self.original_trajectory.poses[goal_index].pose
        self.__set_orientation_from_poses(
            req.request.goal.orientation,
            self.original_trajectory.poses[goal_index],
            self.original_trajectory.poses[goal_index - 1],
        )

        obstacle_array = ObstacleArrayMsg()
        for map_entity in self.map.entities_without_hero():
            x = map_entity.transform.translation().x()
            y = map_entity.transform.translation().y()
            xx, yy = map_entity.to_shapely().poly.exterior.coords.xy
            coordinates = tuple(zip(xx.tolist(), yy.tolist()))

            ob = ObstacleMsg()
            ob.orientation.w = 1
            for c in coordinates:
                x_car, y_car = c
                x, y = self.__car_to_world(np.array([x_car, y_car]))

                ob.polygon.points.append(Point32(x=x, y=y))
                # ob.radius = 1.0
                # if len(ob.polygon.points) == 1:
                #    break

            obstacle_array.obstacles.append(ob)
        req.request.obstacles = obstacle_array

        response: PlanResponse = self.plan_service.call(self.__plan_request)

        print(
            time.time() - start_time, len(obstacle_array.obstacles), "AfterService call"
        )
        response.respond.path.header.frame_id = "global"
        for i in range(len(response.respond.path.poses)):
            response.respond.path.poses[i].header.frame_id = "global"

        beg_idx = (
            self.closest_idx - 2 if self.closest_idx > 2 else 0
        )  # without this merging creates zig zag at boundary

        print(beg_idx)
        self.trajectory.poses = (
            # self.original_trajectory.poses[:beg_idx]  # safety
            self.original_trajectory.poses[:behind_idx]
            + response.respond.path.poses
            + self.original_trajectory.poses[goal_index:]
        )
        print(time.time() - start_time, "len", len(response.respond.path.poses))

    def __set_orientation_from_heading(self, orientation, heading: float):
        (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ) = R.from_euler("z", heading, degrees=False).as_quat()

    def __set_orientation_from_poses(
        self, orientation, pose_a: PoseStamped, pose_b: PoseStamped
    ):
        a = pose_a.pose.position
        b = pose_b.pose.position
        (
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ) = R.from_euler("z", np.arctan2(a.y - b.y, a.x - b.x), degrees=False).as_quat()

    def __car_to_world(self, translation: NDArray) -> NDArray:
        """Input a vector of x and y in car coordinates and transform to world coordinates.
        Args:
            translation (NDArray): 2D Vector in car coordinates

        Returns:
            NDArray: 2D Vector of position in world coordinates
        """
        if self.current_heading is None or self.current_pos is None:
            return translation

        car_phi = self.current_heading
        car_position = self.current_pos[:2]

        rotation_matrix = np.array(
            [[np.cos(car_phi), -np.sin(car_phi)], [np.sin(car_phi), np.cos(car_phi)]]
        )
        world_position = (
            rotation_matrix @ translation + car_position
        )  # Rotate and then translate
        return world_position

    def generate_overtake_trajectory(self, distance, pose_list, unstuck=False):
        """
        Generates a trajectory for overtaking maneuvers.

        This method creates a new trajectory for the vehicle to follow when an
        overtaking maneuver is required. It adjusts the waypoints based on the current
        waypoint and distance, and applies an offset to the waypoints to create a path
        that avoids obstacles or gets the vehicle unstuck.

        Args:
            distance (int): The distance over which the overtaking maneuver should be
                planned.
            pose_list (list): A list of PoseStamped objects representing the current
                planned path.
            unstuck (bool, optional): A flag indicating whether the vehicle is stuck
                and requires a larger offset to get unstuck. Defaults to False.

        Returns:
            None: The method updates the self.trajectory attribute with the new path.
        """
        self._generate_overtake_trajectory(distance)

        return
        currentwp = self.current_wp_idx
        normal_x_offset = 2
        unstuck_x_offset = 3  # could need adjustment with better steering
        if unstuck:
            selection = pose_list[
                int(currentwp) - 2 : int(currentwp) + int(distance) + 2 + NUM_WAYPOINTS
            ]
        else:
            selection = pose_list[
                int(currentwp)
                + int(distance / 2) : int(currentwp)
                + int(distance)
                + NUM_WAYPOINTS
            ]
        waypoints = self.convert_pose_to_array(selection)

        if unstuck is True:
            offset = np.array([unstuck_x_offset, 0, 0])
        else:
            offset = np.array([normal_x_offset, 0, 0])
        rotation_adjusted = Rotation.from_euler(
            "z", self.current_heading + math.radians(90)
        )
        offset_front = rotation_adjusted.apply(offset)
        offset_front = offset_front[:2]
        waypoints_off = waypoints + offset_front

        result_x = waypoints_off[:, 0]
        result_y = waypoints_off[:, 1]

        result = []
        for i in range(len(result_x)):
            position = Point(result_x[i], result_y[i], 0)
            orientation = Quaternion(x=0, y=0, z=0, w=0)
            pose = Pose(position, orientation)
            pos = PoseStamped()
            pos.header.frame_id = "global"
            pos.pose = pose
            result.append(pos)
        path = Path()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = "global"
        if unstuck:
            path.poses = (
                pose_list[: int(currentwp) - 2]
                + result
                + pose_list[int(currentwp) + int(distance) + 2 + NUM_WAYPOINTS :]
            )
        else:
            path.poses = (
                pose_list[: int(currentwp) + int(distance / 2)]
                + result
                + pose_list[int(currentwp + distance + NUM_WAYPOINTS) :]
            )

        # self.trajectory = path

    def __set_trajectory(self, data: Path):
        """get current trajectory global planning

        Args:
            data (Path): Trajectory waypoints
        """
        if self.trajectory is None:
            self.loginfo("Trajectory received")
            self.trajectory = data
            self.original_trajectory = deepcopy(data)
            self.__corners = self.__calc_corner_points()

    def __route_update_callback(self, msg: Empty):
        self.trajectory = None

    def __calc_corner_points(self) -> List[List[np.ndarray]]:
        """
        Calculate the corner points of the trajectory.

        This method converts the poses in the trajectory to an array of coordinates,
        calculates the angles between consecutive points, and identifies the points
        where there is a significant change in the angle, indicating a corner or curve.
        The corner points are then grouped by proximity and returned.

        Returns:
            list: A list of lists, where each sublist contains 2D points that form a
                corner.
        """
        coords = self.convert_pose_to_array(np.array(self.original_trajectory.poses))

        # TODO: refactor by using numpy functions
        x_values = np.array([point[0] for point in coords])
        y_values = np.array([point[1] for point in coords])

        angles = np.arctan2(np.diff(y_values), np.diff(x_values))
        angles = np.rad2deg(angles)
        angles[angles > 0] -= 360  # Convert for angles between 0 - 360 degree

        threshold = 1  # in degree
        curve_change_indices = np.where(np.abs(np.diff(angles)) > threshold)[0]

        sublist = self.group_points_by_proximity(curve_change_indices, proximity=5)

        coords_of_curve = [coords[i] for i in sublist]

        return coords_of_curve

    def group_points_by_proximity(self, points, proximity=5):
        """
        Groups a list of points into sublists based on their proximity to each other.

        Args:
            points (list): A list of points to be grouped.
            proximity (int, optional): The maximum distance between points in a sublist

        Returns:
            list: A list of sublists, where each sublist contains points that are within
                the specified proximity of each other. Sublists with only one point are
                filtered out.
        """
        sublists = []
        current_sublist = []

        for point in points:
            if not current_sublist:
                current_sublist.append(point)
            else:
                last_point = current_sublist[-1]
                distance = abs(point - last_point)

                if distance <= proximity:
                    current_sublist.append(point)
                else:
                    sublists.append(current_sublist)
                    current_sublist = [point]
        if current_sublist:
            sublists.append(current_sublist)

        # TODO: Check if it is intended to filter out sublists with only one point
        filtered_list = [in_list for in_list in sublists if len(in_list) > 1]

        return filtered_list

    def get_cornering_speed(self):
        corner = self.__corners[0]
        pos = self.current_pos[:2]

        def euclid_dist(vector1, vector2):
            # TODO replace with numpy function
            point1 = np.array(vector1)
            point2 = np.array(vector2)

            diff = point2 - point1
            sum_sqrt = np.dot(diff.T, diff)
            return np.sqrt(sum_sqrt)

        def map_corner(dist):
            if dist < 8:  # lane_change
                return 8
            elif dist < 25:
                return 4
            elif dist < 50:
                return 7
            else:
                8  # TODO add return

        distance_corner = 0
        for i in range(len(corner) - 1):
            distance_corner += euclid_dist(corner[i], corner[i + 1])
        # self.loginfo(distance_corner)

        if self.__in_corner:
            distance_end = euclid_dist(pos, corner[0])
            if distance_end > distance_corner + 2:
                self.__in_corner = False
                self.__corners.pop(0)
                self.loginfo("End Corner")
                return self.__get_speed_cruise()
            else:
                return map_corner(distance_corner)

        distance_start = euclid_dist(pos, corner[0])
        if distance_start < 3:
            self.__in_corner = True
            self.loginfo("Start Corner")
            return map_corner(distance_corner)
        else:
            return self.__get_speed_cruise()

    @staticmethod
    def convert_pose_to_array(poses: np.ndarray) -> np.ndarray:
        """Convert an array of PoseStamped objects to a numpy array of positions.

        Args:
            poses (np.ndarray): Array of PoseStamped objects.

        Returns:
            np.ndarray: Numpy array of shape (n, 2) containing the x and y positions.
        """
        result_array = np.empty((len(poses), 2))
        for pose in range(len(poses)):
            result_array[pose] = np.array(
                [poses[pose].pose.position.x, poses[pose].pose.position.y]
            )
        return result_array

    def __check_emergency(self, data: Bool):
        """If an emergency stop is needed first check if we are
        in parking behavior. If we are ignore the emergency stop.

        Args:
            data (Bool): True if emergency stop detected by collision check
        """
        # self.loginfo("Emergency stop detected")
        if not self.__curr_behavior == bs.parking.name:
            # self.loginfo("Emergency stop detected and executed")
            self.emergency_pub.publish(data)

    def update_target_speed(self, acc_speed, behavior):
        """
        Updates the target velocity based on the current behavior and ACC velocity and
        overtake status and publishes it. The unit of the velocity is m/s.
        """
        be_speed = self.get_speed_by_behavior(behavior)
        if behavior == bs.parking.name or self.__overtake_status == 1:
            self.target_speed = be_speed
        else:
            corner_speed = self.get_cornering_speed()
            self.target_speed = min(be_speed, acc_speed, corner_speed)
            if self.target_speed == acc_speed:
                self.target_velocity_selector = "acc_speed"
                # be speed is sometimes equals acc speed (in case of cruise behaviour)
            elif self.target_speed == be_speed:
                self.target_velocity_selector = "be_speed"
            elif self.target_speed == corner_speed:
                self.target_velocity_selector = "corner_speed"
        # self.target_speed = min(self.target_speed, 8)
        self.velocity_pub.publish(self.target_speed)
        self.velocity_selector_pub.publish(self.target_velocity_selector)
        # self.logerr(f"Speed: {self.target_speed}")
        # self.speed_list.append(self.target_speed)

    def __set_acc_speed(self, data: Float32):
        self.__acc_speed = data.data

    def __set_curr_behavior(self, data: String):
        """
        Sets the received current behavior of the vehicle.
        If the behavior is an overtake behavior, a trajectory change is triggered.
        """
        self.__curr_behavior = data.data
        if data.data == bs.ot_enter_init.name:
            if np.isinf(self.__collision_point):
                self.__overtake_status = -1
                self.overtake_success_pub.publish(self.__overtake_status)
                return
            self.change_trajectory(self.__collision_point)

    def __set_stopline(self, data: Waypoint) -> float:
        if data is not None:
            self.__stopline = (data.distance, data.isStopLine)

    def __set_change_point(self, data: LaneChange):
        if data is not None:
            self.__change_point = (data.distance, data.isLaneChange, data.roadOption)

    def __set_collision_point(self, data: Float32MultiArray):
        if data.data is not None:
            self.__collision_point = data.data[0]

    def get_speed_by_behavior(self, behavior: str) -> float:
        speed = 0.0
        split = "_"
        short_behavior = behavior.partition(split)[0]

        if short_behavior == "int":
            speed = self.__get_speed_intersection(behavior)
        elif short_behavior == "lc":
            speed = self.__get_speed_lanechange(behavior)
        elif short_behavior == "ot":
            speed = self.__get_speed_overtake(behavior)
        elif short_behavior == "parking":
            speed = bs.parking.speed
        elif short_behavior == "us":
            speed = self.__get_speed_unstuck(behavior)
        else:
            self.__overtake_status = -1
            speed = self.__get_speed_cruise()
        return speed

    def __get_speed_unstuck(self, behavior: str) -> float:
        # TODO check if this 'global' is necessary
        global UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE
        speed = 0.0
        if behavior == bs.us_unstuck.name:
            speed = bs.us_unstuck.speed
        elif behavior == bs.us_stop.name:
            speed = bs.us_stop.speed
        elif behavior == bs.us_overtake.name:
            pose_list = self.original_trajectory.poses
            if self.unstuck_distance is None:
                self.logfatal("Unstuck distance not set")
                return speed

            if self.init_overtake_pos is not None and self.current_pos is not None:
                distance = np.linalg.norm(
                    self.init_overtake_pos[:2] - self.current_pos[:2]
                )
                # self.logfatal(f"Unstuck Distance in mp: {distance}")
                # clear distance to last unstuck -> avoid spamming overtake
                if distance > UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE:
                    self.unstuck_overtake_flag = False
                    self.logwarn("Unstuck Overtake Flag Cleared")

            # to avoid spamming the overtake_fallback
            if self.unstuck_overtake_flag is False:
                # create overtake trajectory starting 6 meteres before
                # the obstacle
                # 6 worked well in tests, but can be adjusted
                self.generate_overtake_trajectory(
                    self.unstuck_distance, pose_list, unstuck=True
                )
                self.logfatal("Overtake Trajectory while unstuck!")
                self.unstuck_overtake_flag = True
                self.init_overtake_pos = self.current_pos[:2]
            # else: overtake not possible

            speed = bs.us_overtake.speed

        return speed

    def __get_speed_intersection(self, behavior: str) -> float:
        speed = 0.0
        if behavior == bs.int_app_init.name:
            speed = bs.int_app_init.speed
        elif behavior == bs.int_app_green.name:
            speed = bs.int_app_green.speed
        elif behavior == bs.int_app_to_stop.name:
            speed = self.__calc_speed_to_stop_intersection()
        elif behavior == bs.int_wait.name:
            speed == bs.int_wait.speed
        elif behavior == bs.int_enter.name:
            speed = bs.int_enter.speed
        elif behavior == bs.int_exit:
            speed = self.__get_speed_cruise()

        return speed

    def __get_speed_lanechange(self, behavior: str) -> float:
        speed = 0.0
        if behavior == bs.lc_app_init.name:
            speed = bs.lc_app_init.speed
        elif behavior == bs.lc_app_blocked.name:
            speed = self.__calc_speed_to_stop_lanechange()
        elif behavior == bs.lc_app_free.name:
            speed = bs.lc_app_free.speed
        elif behavior == bs.lc_wait.name:
            speed = bs.lc_wait.speed
        elif behavior == bs.lc_enter_init.name:
            speed = bs.lc_enter_init.speed
        elif behavior == bs.lc_exit.name:
            speed = bs.lc_exit.speed

        return speed

    def __get_speed_overtake(self, behavior: str) -> float:
        speed = 0.0
        if behavior == bs.ot_app_blocked.name:
            speed = self.__calc_speed_to_stop_overtake()
        elif behavior == bs.ot_app_free.name:
            speed = self.__calc_speed_to_stop_overtake()
        elif behavior == bs.ot_wait_stopped.name:
            speed = bs.ot_wait_stopped.speed
        elif behavior == bs.ot_wait_free.name:
            speed == self.__get_speed_cruise()
        elif behavior == bs.ot_enter_init.name:
            speed = self.__get_speed_cruise()
        elif behavior == bs.ot_enter_slow.name:
            speed = self.__calc_speed_to_stop_overtake()
        elif behavior == bs.ot_leave.name:
            speed = convert_to_ms(30.0)
        return speed

    def __get_speed_cruise(self) -> float:
        return self.__acc_speed

    def __calc_speed_to_stop_intersection(self) -> float:
        target_distance = TARGET_DISTANCE_TO_STOP
        stopline = self.__calc_virtual_stopline()

        # calculate speed needed for stopping
        v_stop = max(convert_to_ms(10.0), convert_to_ms(stopline / 0.8))
        if v_stop > bs.int_app_init.speed:
            v_stop = bs.int_app_init.speed
        if stopline < target_distance:
            v_stop = 0.0
        return v_stop

    def __calc_speed_to_stop_lanechange(self) -> float:
        stopline = self.__calc_virtual_change_point()

        v_stop = max(convert_to_ms(10.0), convert_to_ms(stopline / 0.8))
        if v_stop > bs.lc_app_init.speed:
            v_stop = bs.lc_app_init.speed
        if stopline < TARGET_DISTANCE_TO_STOP:
            v_stop = 0.0
        return v_stop

    def __calc_speed_to_stop_overtake(self) -> float:
        stopline = self.__calc_virtual_overtake()
        v_stop = max(convert_to_ms(10.0), convert_to_ms(stopline / 0.8))
        if stopline < TARGET_DISTANCE_TO_STOP:
            v_stop = 0.0

        return v_stop

    def __calc_virtual_change_point(self) -> float:
        if self.__change_point[0] != np.inf and self.__change_point[1]:
            return self.__change_point[0]
        else:
            return 0.0

    def __calc_virtual_stopline(self) -> float:
        if self.__stopline[0] != np.inf and self.__stopline[1]:
            stopline = self.__stopline[0]
            if self.traffic_light_y_distance < 250 and stopline > 10:
                return 10
            elif self.traffic_light_y_distance < 180 and stopline > 7:
                return 0.0
            else:
                return stopline
        else:
            return 0.0

    def __calc_virtual_overtake(self) -> float:
        if (self.__collision_point is not None) and self.__collision_point != np.inf:
            return self.__collision_point
        else:
            return 0.0

    def __calculate_closest_trajectory_idx(self):
        if self.current_pos is not None and self.original_trajectory is not None:
            distances = np.array(
                [
                    self.__dist_to(pose.pose.position)
                    for pose in self.original_trajectory.poses
                ]
            )
            self.closest_idx = int(np.argmin(distances))
            self.distance_to_trajectory = distances[self.closest_idx]

    def __dist_to(self, pos: Point) -> float:
        """
        Distance between current position and target position (only (x,y))
        :param pos: targeted position
        :return: distance
        """
        x_current = self.current_pos[0]
        y_current = self.current_pos[1]
        x_target = pos.x
        y_target = pos.y
        d = (x_target - x_current) ** 2 + (y_target - y_current) ** 2
        return math.sqrt(d)

    def run(self):
        """
        Control loop that updates the target speed and publishes the target trajectory
        and speed over ROS topics.
        """

        def loop(timer_event=None):
            self.__calculate_closest_trajectory_idx()

            if (
                self.__curr_behavior is not None
                and self.__acc_speed is not None
                and self.__corners is not None
            ):
                self.trajectory.header.stamp = rospy.Time.now()
                self._generate_overtake_trajectory(
                    None
                )  # This is a decision we need to make
                self.traj_pub.publish(self.trajectory)
                self.update_target_speed(self.__acc_speed, self.__curr_behavior)
            else:
                self.velocity_pub.publish(0.0)
                self.velocity_selector_pub.publish("not selected")

        self.new_timer(self.control_loop_rate, loop)
        self.spin()


if __name__ == "__main__":
    roscomp.init("MotionPlanning")
    try:
        node = MotionPlanning()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

#!/usr/bin/env python
import math
from typing import List, Optional

import numpy as np
import shapely
from shapely import LineString

import ros_compatibility as roscomp
import rospy
from carla_msgs.msg import CarlaSpeedometer
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Path
from perception.msg import LaneChange, Waypoint
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16, String
from nav_msgs.msg import Path
from planning.srv import (
    StartOvertake,
    EndOvertake,
    OvertakeStatus,
    StartOvertakeRequest,
    EndOvertakeRequest,
    OvertakeStatusRequest,
    StartOvertakeResponse,
    EndOvertakeResponse,
    OvertakeStatusResponse,
)

import mapping_common.hero
import mapping_common.mask
import mapping_common.map
from mapping_common.transform import Vector2, Point2, Transform2D

from local_planner.utils import (
    NUM_WAYPOINTS,
    NUM_WAYPOINTS_BICYCLE,
    TARGET_DISTANCE_TO_STOP,
    TARGET_DISTANCE_TO_STOP_OVERTAKE,
    convert_to_ms,
    spawn_car,
    convert_pose_to_array,
)
from behavior_agent.behaviors import behavior_speed as bs


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

        # TODO: add type hints
        # self.target_speed = 0.0
        # self.target_velocity_selector = "not_selected"
        # self.__curr_behavior = None
        # self.__acc_speed = 0.0
        # self.__stopline = None  # (Distance, isStopline)
        # self.__change_point = None  # (Distance, isLaneChange, roadOption)
        # TODO: clarify what the overtake_status values mean (by using an enum or ...)
        # self.__overtake_status = -1
        # self.published = False

        # Overtake related stuff
        self.overtake_request: Optional[StartOvertakeRequest] = None
        """If an overtake is queued or running, this is populated
        """
        self.overtake_status: OvertakeStatusResponse = OvertakeStatusResponse(
            status=OvertakeStatusResponse.NO_OVERTAKE
        )

        # Trajectory related stuff
        self.current_pos: Optional[Point2] = None
        self.current_heading: Optional[float] = None
        self.global_trajectory: Optional[Path] = None
        self.init_trajectory: bool = True
        """If the trajectory has changed and we need to do initialization"""
        self.speed_limits_OD = None
        self.current_global_waypoint_idx: int = 0
        """Index of the !!ROUGHLY!! current waypoint of self.global_trajectory

        Look in self._update_current_global_waypoint_idx() for why it even exists
        """
        # self.overtaking = False
        # self.enhanced_path = None
        # self.current_speed = None
        # self.speed_limit = None
        # self.__corners = None
        # self.__in_corner = False
        # self.calculated = False
        # self.traffic_light_y_distance = np.inf
        # # unstuck routine variables
        # self.unstuck_distance = None
        # self.unstuck_overtake_flag = False
        # self.init_overtake_pos = None
        # self.__ot_bicycle = False

        # Subscriber
        self.test_sub = self.new_subscription(
            Float32, f"/paf/{self.role_name}/spawn_car", spawn_car, qos_profile=1
        )
        # self.speed_limit_sub = self.new_subscription(
        #     Float32,
        #     f"/paf/{self.role_name}/speed_limit",
        #     self.__set_speed_limit,
        #     qos_profile=1,
        # )
        # self.velocity_sub: Subscriber = self.new_subscription(
        #     CarlaSpeedometer,
        #     f"/carla/{self.role_name}/Speed",
        #     self.__get_current_velocity,
        #     qos_profile=1,
        # )
        self.head_sub = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__set_heading,
            qos_profile=1,
        )

        self.trajectory_sub = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory_global",
            self.__set_global_trajectory,
            qos_profile=1,
        )
        # Get initial set of speed limits from global planner
        self.speed_limit_OD_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/speed_limits_OpenDrive",
            self.__set_speed_limits_opendrive,
            qos_profile=1,
        )
        self.current_pos_sub = self.new_subscription(
            PoseStamped,
            f"/paf/{self.role_name}/current_pos",
            self.__set_current_pos,
            qos_profile=1,
        )
        # self.curr_behavior_sub: Subscriber = self.new_subscription(
        #     String,
        #     f"/paf/{self.role_name}/curr_behavior",
        #     self.__set_curr_behavior,
        #     qos_profile=1,
        # )
        # self.ot_distance_sub: Subscriber = self.new_subscription(
        #     Float32,
        #     f"/paf/{self.role_name}/overtake_distance",
        #     self.__set_ot_distance,
        #     qos_profile=1,
        # )
        # self.emergency_sub: Subscriber = self.new_subscription(
        #     Bool,
        #     f"/paf/{self.role_name}/unchecked_emergency",
        #     self.__check_emergency,
        #     qos_profile=1,
        # )
        # self.acc_sub: Subscriber = self.new_subscription(
        #     Float32,
        #     f"/paf/{self.role_name}/acc_velocity",
        #     self.__set_acc_speed,
        #     qos_profile=1,
        # )

        # self.stopline_sub: Subscriber = self.new_subscription(
        #     Waypoint,
        #     f"/paf/{self.role_name}/waypoint_distance",
        #     self.__set_stopline,
        #     qos_profile=1,
        # )

        # self.change_point_sub: Subscriber = self.new_subscription(
        #     LaneChange,
        #     f"/paf/{self.role_name}/lane_change_distance",
        #     self.__set_change_point,
        #     qos_profile=1,
        # )

        # self.traffic_y_sub: Subscriber = self.new_subscription(
        #     Int16,
        #     f"/paf/{self.role_name}/Center/traffic_light_y_distance",
        #     self.__set_traffic_y_distance,
        #     qos_profile=1,
        # )
        # self.unstuck_distance_sub: Subscriber = self.new_subscription(
        #     Float32,
        #     f"/paf/{self.role_name}/unstuck_distance",
        #     self.__set_unstuck_distance,
        #     qos_profile=1,
        # )

        # self.ot_bicycle_sub: Subscriber = self.new_subscription(
        #     Bool,
        #     f"/paf/{self.role_name}/ot_bicycle",
        #     self.__set_ot_bicycle,
        #     qos_profile=1,
        # )

        # Services

        self.start_overtake_service = rospy.Service(
            f"/paf/{self.role_name}/motion_planning/start_overtake",
            StartOvertake,
            self.__start_overtake_callback,
        )

        self.end_overtake_service = rospy.Service(
            f"/paf/{self.role_name}/motion_planning/end_overtake",
            EndOvertake,
            self.__end_overtake_callback,
        )

        self.overtake_status_service = rospy.Service(
            f"/paf/{self.role_name}/motion_planning/overtake_status",
            OvertakeStatus,
            self.__overtake_status_callback,
        )

        # Publisher

        self.global_trajectory_pub: Publisher = self.new_publisher(
            msg_type=Path, topic=f"/paf/{self.role_name}/trajectory", qos_profile=1
        )
        self.local_trajectory_pub: Publisher = self.new_publisher(
            msg_type=Path,
            topic=f"/paf/{self.role_name}/trajectory_local",
            qos_profile=1,
        )
        # self.velocity_pub: Publisher = self.new_publisher(
        #     Float32, f"/paf/{self.role_name}/target_velocity", qos_profile=1
        # )
        self.speed_limit_publisher: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/speed_limit", qos_profile=1
        )

        # self.velocity_selector_pub: Publisher = self.new_publisher(
        #     String, f"/paf/{self.role_name}/target_velocity_selector", qos_profile=1
        # )

        self.logdebug("MotionPlanning started")
        self.counter = 0

    def __start_overtake_callback(
        self, req: StartOvertakeRequest
    ) -> StartOvertakeResponse:
        msg = "Received request for overtake"
        if self.overtake_request is not None:
            if sign(req.offset) == sign(self.overtake_request.offset):
                # Offsets have the same sign -> take the abs bigger one
                req.offset = max(
                    abs(req.offset), abs(self.overtake_request.offset)
                ) * sign(req.offset)
                msg = f"Adjusted existing overtake. Used bigger offset: {req.offset}"
            else:
                msg = "Overwrote existing overtake."

        if self.overtake_status.status == OvertakeStatusResponse.NO_OVERTAKE:
            self.overtake_status.status = OvertakeStatusResponse.OVERTAKE_QUEUED

        self.overtake_request = req

        rospy.loginfo(f"MotionPlanning: {msg}")
        return StartOvertakeResponse(success=True, msg=msg)

    def __end_overtake_callback(self, req: EndOvertakeRequest) -> EndOvertakeResponse:
        msg = "No overtake aborted: No overtake has been running"
        if self.overtake_request is not None:
            if req.has_end_pos:
                self.overtake_request.has_end_pos = True
                self.overtake_request.end_pos = req.end_pos
                self.overtake_request.transition_length = req.transition_length
                msg = "Adjusted end of existing overtake"
            else:
                self.overtake_request = None
                msg = "Cancelled existing overtake"
                self.overtake_status.status = OvertakeStatusResponse.NO_OVERTAKE

        rospy.loginfo(f"MotionPlanning: {msg}")
        return EndOvertakeResponse(success=True, msg=msg)

    def __overtake_status_callback(
        self, req: OvertakeStatusRequest
    ) -> OvertakeStatusResponse:
        return self.overtake_status

    # def __set_unstuck_distance(self, data: Float32):
    #     """Set unstuck distance

    #     Args:
    #         data (Float32): Unstuck distance
    #     """
    #     self.unstuck_distance = data.data

    # def __set_speed_limit(self, data: Float32):
    #     """Set current speed limit

    #     Args:
    #         data (Float32): Current speed limit
    #     """
    #     self.speed_limit = data.data

    # def __get_current_velocity(self, data: CarlaSpeedometer):
    #     """Get current velocity from CarlaSpeedometer

    #     Args:
    #         data (CarlaSpeedometer): Current velocity
    #     """
    #     self.current_speed = float(data.speed)

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
        self.current_pos = Point2.from_ros_msg(data.pose.position)
        self.publish_local_trajectory()

    def publish_local_trajectory(self):
        if (
            self.current_heading is None
            or self.global_trajectory is None
            or self.current_pos is None
        ):
            return

        hero_transform = mapping_common.map.build_global_hero_transform(
            self.current_pos.x(),
            self.current_pos.y(),
            self.current_heading,
        )

        if self.init_trajectory:
            # We need to find the start point first
            self.current_global_waypoint_idx = 0
            max_length = None
            self.init_trajectory = False
        else:
            self._update_current_global_waypoint_idx()
            max_length = 200.0

        local_trajectory = mapping_common.mask.build_trajectory(
            self.global_trajectory,
            hero_transform,
            max_length=max_length,
            current_wp_idx=max(0, self.current_global_waypoint_idx - 2),
            max_wp_count=None if max_length is None else int(max_length * 2),
            centered=False,
        )
        if local_trajectory is None:
            # Try to reinitialize trajectory on next position update
            self.init_trajectory = True
            rospy.logfatal_throttle(1.0, "MotionPlanning: Empty trajectory")
            return

        (start_x, start_y) = local_trajectory.coords[0]
        start_vector = Vector2.new(start_x, start_y)
        if start_vector.length() > 25.0:
            # We are far away from the trajectory
            # Try to reinitialize trajectory on next position update
            self.init_trajectory = True
            rospy.logfatal_throttle(1.0, "MotionPlanning: Too far away from trajectory")
            return

        local_trajectory = self._apply_overtake(
            local_trajectory, hero_transform=hero_transform
        )

        # Calculation finished, ready for publishing
        local_path = mapping_common.mask.line_to_ros_path(local_trajectory)

        local_path.header.stamp = rospy.get_rostime()
        local_path.header.frame_id = "hero"
        # Publish local path
        self.local_trajectory_pub.publish(local_path)
        # Publish the global path for reference
        self.global_trajectory.header.stamp = rospy.get_rostime()
        self.global_trajectory.header.frame_id = "global"
        self.global_trajectory_pub.publish(self.global_trajectory)
        # Publish speed limit
        if (
            self.speed_limits_OD is not None
            and len(self.speed_limits_OD) > self.current_global_waypoint_idx
        ):
            self.speed_limit_publisher.publish(
                self.speed_limits_OD[self.current_global_waypoint_idx]
            )
        else:
            rospy.logwarn_throttle(
                1, "Motion planning: No speed limit available for current waypoint"
            )

    def _update_current_global_waypoint_idx(self):
        """The next_global_waypoint_idx is used to limit the build_trajectory
        algorithm to a certain part of the trajectory.
        This saves processing time and
        avoids bugs with a "looping"/self-crossing trajectory"""
        if len(self.global_trajectory.poses) > self.current_global_waypoint_idx + 1:
            pose0: Pose = self.global_trajectory.poses[
                self.current_global_waypoint_idx
            ].pose
            pose1: Pose = self.global_trajectory.poses[
                self.current_global_waypoint_idx + 1
            ].pose
            point0 = Point2.from_ros_msg(pose0.position)
            point1 = Point2.from_ros_msg(pose1.position)
            if self.current_pos.distance_to(point0) > self.current_pos.distance_to(
                point1
            ):
                self.current_global_waypoint_idx += 1
            elif self.current_global_waypoint_idx > 0:
                last_pose: Pose = self.global_trajectory.poses[
                    self.current_global_waypoint_idx - 1
                ].pose
                last_point = Point2.from_ros_msg(last_pose.position)
                if self.current_pos.distance_to(point0) > self.current_pos.distance_to(
                    last_point
                ):
                    self.current_global_waypoint_idx -= 1

    def _apply_overtake(
        self, local_trajectory: LineString, hero_transform: Transform2D
    ) -> LineString:
        if self.overtake_request is None:
            self.overtake_status.status = OvertakeStatusResponse.NO_OVERTAKE
            return local_trajectory

        hero_transform_inverse = hero_transform.inverse()

        before_trajectory: Optional[LineString] = None
        overtake_trajectory: Optional[LineString] = local_trajectory
        after_trajectory: Optional[LineString] = None

        overtake_request = self.overtake_request
        if overtake_request.has_start_pos:
            local_start_point: Point2 = hero_transform_inverse * Point2.from_ros_msg(
                overtake_request.start_pos
            )
            local_start_point_s = local_start_point.to_shapely()
            start_dist = overtake_trajectory.line_locate_point(local_start_point_s)
            (before_trajectory, overtake_trajectory) = (
                mapping_common.mask.split_line_at(overtake_trajectory, start_dist)
            )
        if overtake_request.has_end_pos and overtake_trajectory is not None:
            local_end_point: Point2 = hero_transform_inverse * Point2.from_ros_msg(
                overtake_request.end_pos
            )
            local_end_point_s = local_end_point.to_shapely()
            end_dist = overtake_trajectory.line_locate_point(local_end_point_s)
            (overtake_trajectory, after_trajectory) = mapping_common.mask.split_line_at(
                overtake_trajectory, end_dist
            )

            if overtake_trajectory is None:
                # If we are after the end of the overtake -> delete overtake_request
                rospy.loginfo("MotionPlanning: Finished overtake")
                self.overtake_request = None
                self.overtake_status.status = OvertakeStatusResponse.NO_OVERTAKE

        if overtake_trajectory is None:
            return local_trajectory

        # Create the overtake by offsetting
        overtake_trajectory = overtake_trajectory.offset_curve(
            distance=overtake_request.offset
        )

        # Apply "smooth" transition by cropping the before and after parts
        transition_length = overtake_request.transition_length
        if before_trajectory is not None:
            self.overtake_status.status = OvertakeStatusResponse.OVERTAKE_QUEUED
        else:
            # We only start overtaking if we are
            # close enough to the overtake trajectory.
            # In local coordinated the position of the car is (0, 0), but
            # Using the front (hood) position for the check is better
            hero = mapping_common.hero.create_hero_entity()
            front_point_s = shapely.Point(hero.get_front_x(), 0.0)
            distance_to_overtake = shapely.distance(overtake_trajectory, front_point_s)
            if distance_to_overtake < 0.5:
                self.overtake_status.status = OvertakeStatusResponse.OVERTAKING

        ot_length = overtake_trajectory.length
        overtake_trajectory_cropped = mapping_common.mask.clamp_line(
            overtake_trajectory, transition_length, ot_length - transition_length
        )
        if overtake_trajectory_cropped is not None:
            overtake_trajectory = overtake_trajectory_cropped

        # Build final trajectory by appending coordinates
        coords = np.array(overtake_trajectory.coords)
        if before_trajectory is not None:
            coords = np.append(np.array(before_trajectory.coords), coords)
        if after_trajectory is not None:
            coords = np.append(coords, np.array(after_trajectory.coords))

        overtake_trajectory = LineString(coords)
        return overtake_trajectory

    # def __set_traffic_y_distance(self, data):
    #     if data is not None:
    #         self.traffic_light_y_distance = data.data

    # def change_trajectory(self, distance_obj):
    #     """update trajectory for overtaking and convert it
    #     to a new Path message

    #     Args:
    #         distance_obj (float): distance to overtake object
    #     """
    #     pose_list = self.global_trajectory.poses

    #     # Only use fallback
    #     # self.generate_overtake_trajectory(distance_obj, pose_list)
    #     self.__overtake_status = 1
    #     self.overtake_success_pub.publish(self.__overtake_status)
    #     return

    # def generate_overtake_trajectory(self, distance, pose_list, unstuck=False):
    #     """
    #     Generates a trajectory for overtaking maneuvers.

    #     This method creates a new trajectory for the vehicle to follow when an
    #     overtaking maneuver is required. It adjusts the waypoints based on the current
    #     waypoint and distance, and applies an offset to the waypoints to create a path
    #     that avoids obstacles or gets the vehicle unstuck.

    #     Args:
    #         distance (int): The distance over which the overtaking maneuver should be
    #             planned.
    #         pose_list (list): A list of PoseStamped objects representing the current
    #             planned path.
    #         unstuck (bool, optional): A flag indicating whether the vehicle is stuck
    #             and requires a larger offset to get unstuck. Defaults to False.

    #     Returns:
    #         None: The method updates the self.trajectory attribute with the new path.
    #     """
    #     if self.__ot_bicycle:
    #         waypoints_num = NUM_WAYPOINTS_BICYCLE
    #     else:
    #         waypoints_num = NUM_WAYPOINTS

    #     normal_x_offset = 2
    #     unstuck_x_offset = 3  # could need adjustment with better steering
    #     if unstuck:
    #         selection = pose_list[
    #             int(currentwp) - 2 : int(currentwp) + int(distance) + 2 + waypoints_num
    #         ]
    #     else:
    #         selection = pose_list[
    #             int(currentwp)
    #             + min(2, int(distance / 2)) : int(currentwp)
    #             + int(distance)
    #             + waypoints_num
    #             + 2
    #         ]
    #     waypoints = convert_pose_to_array(selection)

    #     if unstuck is True:
    #         offset = np.array([unstuck_x_offset, 0, 0])
    #     else:
    #         offset = np.array([normal_x_offset, 0, 0])
    #     rotation_adjusted = Rotation.from_euler(
    #         "z", self.current_heading + math.radians(90)
    #     )
    #     offset_front = rotation_adjusted.apply(offset)
    #     offset_front = offset_front[:2]
    #     waypoints_off = waypoints + offset_front

    #     result_x = waypoints_off[:, 0]
    #     result_y = waypoints_off[:, 1]

    #     result = []
    #     for i in range(len(result_x)):
    #         position = Point(result_x[i], result_y[i], 0)
    #         orientation = Quaternion(x=0, y=0, z=0, w=0)
    #         pose = Pose(position, orientation)
    #         pos = PoseStamped()
    #         pos.header.frame_id = "global"
    #         pos.pose = pose
    #         result.append(pos)
    #     path = Path()
    #     path.header.stamp = rospy.Time.now()
    #     path.header.frame_id = "global"
    #     if unstuck:
    #         path.poses = (
    #             pose_list[: int(currentwp) - 2]
    #             + result
    #             + pose_list[int(currentwp) + int(distance) + 2 + waypoints_num :]
    #         )
    #     else:
    #         path.poses = (
    #             pose_list[: int(currentwp) + min(2, int(distance / 2))]
    #             + result
    #             + pose_list[int(currentwp + distance + waypoints_num + 2) :]
    #         )

    #     self.global_trajectory = path

    def __set_global_trajectory(self, data: Path):
        """get current trajectory global planning

        Args:
            data (Path): Trajectory waypoints
        """
        self.global_trajectory = data
        # TODO: Only reinit if we receive a different trajectory
        self.init_trajectory = True
        self.loginfo("Global trajectory received")

    def __set_speed_limits_opendrive(self, data: Float32MultiArray):
        """Recieve speed limits from OpenDrive via global planner

        Args:
            data (Float32MultiArray): speed limits per waypoint
        """
        self.speed_limits_OD = data.data

    # def __calc_corner_points(self) -> List[List[np.ndarray]]:
    #     """
    #     Calculate the corner points of the trajectory.

    #     This method converts the poses in the trajectory to an array of coordinates,
    #     calculates the angles between consecutive points, and identifies the points
    #     where there is a significant change in the angle, indicating a corner or curve.
    #     The corner points are then grouped by proximity and returned.

    #     Returns:
    #         list: A list of lists, where each sublist contains 2D points that form a
    #             corner.
    #     """
    #     coords = convert_pose_to_array(np.array(self.global_trajectory.poses))

    #     # TODO: refactor by using numpy functions
    #     x_values = np.array([point[0] for point in coords])
    #     y_values = np.array([point[1] for point in coords])

    #     angles = np.arctan2(np.diff(y_values), np.diff(x_values))
    #     angles = np.rad2deg(angles)
    #     angles[angles > 0] -= 360  # Convert for angles between 0 - 360 degree

    #     threshold = 1  # in degree
    #     curve_change_indices = np.where(np.abs(np.diff(angles)) > threshold)[0]

    #     sublist = self.group_points_by_proximity(curve_change_indices, proximity=5)

    #     coords_of_curve = [coords[i] for i in sublist]

    #     return coords_of_curve

    # def group_points_by_proximity(self, points, proximity=5):
    #     """
    #     Groups a list of points into sublists based on their proximity to each other.

    #     Args:
    #         points (list): A list of points to be grouped.
    #         proximity (int, optional): The maximum distance between points in a sublist

    #     Returns:
    #         list: A list of sublists, where each sublist contains points that are within
    #             the specified proximity of each other. Sublists with only one point are
    #             filtered out.
    #     """
    #     sublists = []
    #     current_sublist = []

    #     for point in points:
    #         if not current_sublist:
    #             current_sublist.append(point)
    #         else:
    #             last_point = current_sublist[-1]
    #             distance = abs(point - last_point)

    #             if distance <= proximity:
    #                 current_sublist.append(point)
    #             else:
    #                 sublists.append(current_sublist)
    #                 current_sublist = [point]
    #     if current_sublist:
    #         sublists.append(current_sublist)

    #     # TODO: Check if it is intended to filter out sublists with only one point
    #     filtered_list = [in_list for in_list in sublists if len(in_list) > 1]

    #     return filtered_list

    # def get_cornering_speed(self) -> Optional[float]:
    #     if len(self.__corners) < 1:
    #         return None
    #     corner = self.__corners[0]
    #     pos = self.current_pos[:2]

    #     def euclid_dist(vector1, vector2):
    #         # TODO replace with numpy function
    #         point1 = np.array(vector1)
    #         point2 = np.array(vector2)

    #         diff = point2 - point1
    #         sum_sqrt = np.dot(diff.T, diff)
    #         return np.sqrt(sum_sqrt)

    #     def map_corner(dist):
    #         if dist < 8:  # lane_change
    #             return 8
    #         elif dist < 25:
    #             return 4
    #         elif dist < 50:
    #             return 7
    #         else:
    #             8  # TODO add return

    #     distance_corner = 0
    #     for i in range(len(corner) - 1):
    #         distance_corner += euclid_dist(corner[i], corner[i + 1])
    #     # self.loginfo(distance_corner)

    #     if self.__in_corner:
    #         distance_end = euclid_dist(pos, corner[0])
    #         if distance_end > distance_corner + 2:
    #             self.__in_corner = False
    #             self.__corners.pop(0)
    #             self.loginfo("End Corner")
    #             return self.__get_speed_cruise()
    #         else:
    #             return map_corner(distance_corner)

    #     distance_start = euclid_dist(pos, corner[0])
    #     if distance_start < 3:
    #         self.__in_corner = True
    #         self.loginfo("Start Corner")
    #         return map_corner(distance_corner)
    #     else:
    #         return self.__get_speed_cruise()

    # def update_target_speed(self, acc_speed, behavior):
    #     """
    #     Updates the target velocity based on the current behavior and ACC velocity and
    #     overtake status and publishes it. The unit of the velocity is m/s.
    #     """
    #     be_speed = self.get_speed_by_behavior(behavior)
    #     if behavior == bs.parking.name:  # or self.__overtake_status == 1:
    #         self.target_speed = be_speed
    #         self.target_velocity_selector = "be_speed"
    #     else:
    #         corner_speed = self.get_cornering_speed()
    #         speed_list = []
    #         if be_speed is not None:
    #             speed_list.append(be_speed)
    #         if acc_speed is not None:
    #             speed_list.append(acc_speed)
    #         if corner_speed is not None:
    #             speed_list.append(corner_speed)

    #         if len(speed_list) > 0:
    #             self.target_speed = min(speed_list)
    #             if self.target_speed == acc_speed:
    #                 self.target_velocity_selector = "acc_speed"
    #                 # be speed is sometimes equals acc speed (in case of cruise
    #                 # behaviour)
    #             elif self.target_speed == be_speed:
    #                 self.target_velocity_selector = "be_speed"
    #             elif self.target_speed == corner_speed:
    #                 self.target_velocity_selector = "corner_speed"
    #         else:
    #             self.target_speed = 0.0
    #             self.target_velocity_selector = "not_selected"

    #     self.velocity_pub.publish(self.target_speed)
    #     self.velocity_selector_pub.publish(self.target_velocity_selector)
    #     # self.logerr(f"Speed: {self.target_speed}")
    #     # self.speed_list.append(self.target_speed)

    # def __set_acc_speed(self, data: Float32):
    #     self.__acc_speed = data.data

    # def __set_curr_behavior(self, data: String):
    #     """
    #     Sets the received current behavior of the vehicle.
    #     If the behavior is an overtake behavior, a trajectory change is triggered.
    #     """
    #     self.__curr_behavior = data.data
    #     if data.data == bs.ot_enter_init.name:
    #         if np.isinf(self.__ot_distance):
    #             self.__overtake_status = -1
    #             self.overtake_success_pub.publish(self.__overtake_status)
    #             return
    #         # self.change_trajectory(self.__ot_distance)

    # def get_speed_by_behavior(self, behavior: str) -> float:
    #     speed = 0.0
    #     split = "_"
    #     short_behavior = behavior.partition(split)[0]

    #     if short_behavior == "int":
    #         speed = self.__get_speed_intersection(behavior)
    #     elif short_behavior == "lc":
    #         speed = self.__get_speed_lanechange(behavior)
    #     elif short_behavior == "ot":
    #         speed = self.__get_speed_overtake(behavior)
    #     elif short_behavior == "parking":
    #         speed = bs.parking.speed
    #     elif short_behavior == "us":
    #         speed = self.__get_speed_unstuck(behavior)
    #     else:
    #         self.__overtake_status = -1
    #         speed = self.__get_speed_cruise()
    #     return speed

    # def __get_speed_unstuck(self, behavior: str) -> float:
    #     # TODO check if this 'global' is necessary
    #     global UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE
    #     speed = 0.0
    #     if behavior == bs.us_unstuck.name:
    #         speed = bs.us_unstuck.speed
    #     elif behavior == bs.us_stop.name:
    #         speed = bs.us_stop.speed
    #     elif behavior == bs.us_overtake.name:
    #         pose_list = self.global_trajectory.poses
    #         if self.unstuck_distance is None:
    #             self.logfatal("Unstuck distance not set")
    #             return speed

    #         if self.init_overtake_pos is not None and self.current_pos is not None:
    #             distance = np.linalg.norm(
    #                 self.init_overtake_pos[:2] - self.current_pos[:2]
    #             )
    #             # self.logfatal(f"Unstuck Distance in mp: {distance}")
    #             # clear distance to last unstuck -> avoid spamming overtake
    #             if distance > UNSTUCK_OVERTAKE_FLAG_CLEAR_DISTANCE:
    #                 self.unstuck_overtake_flag = False
    #                 self.logwarn("Unstuck Overtake Flag Cleared")

    #         # to avoid spamming the overtake_fallback
    #         if self.unstuck_overtake_flag is False:
    #             # create overtake trajectory starting 6 meteres before
    #             # the obstacle
    #             # 6 worked well in tests, but can be adjusted
    #             # self.generate_overtake_trajectory(
    #             #     self.unstuck_distance, pose_list, unstuck=True
    #             # )
    #             self.logfatal("Overtake Trajectory while unstuck!")
    #             self.unstuck_overtake_flag = True
    #             self.init_overtake_pos = self.current_pos[:2]
    #         # else: overtake not possible

    #         speed = bs.us_overtake.speed

    #     return speed

    # def __get_speed_intersection(self, behavior: str) -> float:
    #     speed = 0.0
    #     if behavior == bs.int_app_init.name:
    #         speed = bs.int_app_init.speed
    #     elif behavior == bs.int_app_green.name:
    #         speed = bs.int_app_green.speed
    #     elif behavior == bs.int_app_to_stop.name:
    #         speed = self.__calc_speed_to_stop_intersection()
    #     elif behavior == bs.int_wait.name:
    #         speed == bs.int_wait.speed
    #     elif behavior == bs.int_enter.name:
    #         speed = bs.int_enter.speed
    #     elif behavior == bs.int_exit:
    #         speed = self.__get_speed_cruise()

    #     return speed

    # def __get_speed_lanechange(self, behavior: str) -> float:
    #     speed = 0.0
    #     if behavior == bs.lc_app_init.name:
    #         speed = bs.lc_app_init.speed
    #     elif behavior == bs.lc_app_blocked.name:
    #         speed = self.__calc_speed_to_stop_lanechange()
    #     elif behavior == bs.lc_app_free.name:
    #         speed = bs.lc_app_free.speed
    #     elif behavior == bs.lc_wait.name:
    #         speed = bs.lc_wait.speed
    #     elif behavior == bs.lc_enter_init.name:
    #         speed = bs.lc_enter_init.speed
    #     elif behavior == bs.lc_exit.name:
    #         speed = bs.lc_exit.speed

    #     return speed

    # def __get_speed_overtake(self, behavior: str) -> float:
    #     speed = 0.0
    #     if behavior == bs.ot_app_blocked.name:
    #         speed = self.__calc_speed_to_stop_overtake()
    #     elif behavior == bs.ot_app_free.name:
    #         speed = self.__calc_speed_to_stop_overtake()
    #     elif behavior == bs.ot_wait_free.name:
    #         speed = bs.ot_wait_free.speed
    #     elif behavior == bs.ot_wait_bicycle.name:
    #         speed = self.__get_speed_cruise()
    #     elif behavior == bs.ot_enter_init.name:
    #         speed = self.__get_speed_cruise()
    #     elif behavior == bs.ot_enter_slow.name:
    #         speed = self.__calc_speed_to_stop_overtake()
    #     elif behavior == bs.ot_leave.name:
    #         speed = convert_to_ms(30.0)
    #     return speed

    # def __get_speed_cruise(self) -> float:
    #     return self.__acc_speed

    # def __calc_speed_to_stop_intersection(self) -> float:
    #     target_distance = TARGET_DISTANCE_TO_STOP
    #     stopline = self.__calc_virtual_stopline()

    #     # calculate speed needed for stopping
    #     v_stop = max(convert_to_ms(10.0), convert_to_ms(stopline / 0.8))
    #     if v_stop > bs.int_app_init.speed:
    #         v_stop = bs.int_app_init.speed
    #     if stopline < target_distance:
    #         v_stop = 0.0
    #     return v_stop

    # def __calc_speed_to_stop_lanechange(self) -> float:
    #     stopline = self.__calc_virtual_change_point()

    #     v_stop = max(convert_to_ms(10.0), convert_to_ms(stopline / 0.8))
    #     if v_stop > bs.lc_app_init.speed:
    #         v_stop = bs.lc_app_init.speed
    #     if stopline < TARGET_DISTANCE_TO_STOP:
    #         v_stop = 0.0
    #     return v_stop

    # def __calc_speed_to_stop_overtake(self) -> float:
    #     stopline = self.__calc_virtual_overtake()
    #     v_stop = min(convert_to_ms(9.0), convert_to_ms(stopline / 1.4))
    #     if stopline < TARGET_DISTANCE_TO_STOP_OVERTAKE:
    #         v_stop = 2.5

    #     return v_stop

    # def __calc_virtual_change_point(self) -> float:
    #     if self.__change_point[0] != np.inf and self.__change_point[1]:
    #         return self.__change_point[0]
    #     else:
    #         return 0.0

    # def __calc_virtual_stopline(self) -> float:
    #     if self.__stopline[0] != np.inf and self.__stopline[1]:
    #         stopline = self.__stopline[0]
    #         if self.traffic_light_y_distance < 250 and stopline > 10:
    #             return 10
    #         elif self.traffic_light_y_distance < 180 and stopline > 7:
    #             return 0.0
    #         else:
    #             return stopline
    #     else:
    #         return 0.0

    # def __calc_virtual_overtake(self) -> float:
    #     if (self.__ot_distance is not None) and self.__ot_distance != np.inf:
    #         return self.__ot_distance - 2.5
    #     else:
    #         return 0.0

    def run(self):
        self.spin()


def sign(f: float) -> float:
    """Returns +-1.0 depending on the sign of f

    Python math has no sign function

    Args:
        f (float)

    Returns:
        float: -1.0 or +1.0 depending on the sign of f
    """
    return math.copysign(1.0, f)


if __name__ == "__main__":
    roscomp.init("MotionPlanning")
    try:
        node = MotionPlanning()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

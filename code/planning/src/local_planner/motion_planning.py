#!/usr/bin/env python
import math
from typing import Optional

import numpy as np
import shapely
from shapely import LineString

import ros_compatibility as roscomp
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
from rospy import Publisher, Subscriber
from std_msgs.msg import Float32, Float32MultiArray
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
from scipy.spatial.transform import Rotation
from std_msgs.msg import Bool, Float32, Float32MultiArray, Int16, String

from local_planner.utils import (
    NUM_WAYPOINTS,
    NUM_WAYPOINTS_BICYCLE,
    TARGET_DISTANCE_TO_STOP,
    TARGET_DISTANCE_TO_STOP_INTERSECTION,
    TARGET_DISTANCE_TO_STOP_OVERTAKE,
    convert_to_ms,
    spawn_car,
    convert_pose_to_array,
)

import mapping_common.hero
import mapping_common.mask
import mapping_common.map
from mapping_common.transform import Vector2, Point2, Transform2D

TRAJECTORY_DISTANCE_THRESHOLD: float = 0.5
"""threshold under which the planner decides it is on a trajectory
"""


class MotionPlanning(CompatibleNode):
    """
    This node received the trajectory_global from the PrePlanner/global_planner.

    It then calculates the trajectory_local and publishes it.

    The published trajectory_local has local coordinates in relation to the hero car
    and is cropped to the projected center point of the car.
    It might also contain an overtake depending on self.overtake_request.

    For starting and stopping overtakes, it provides
    StartOvertake, EndOvertake and OvertakeStatus services

    It also publishes the OpenDrive speed limit based on the hero's position
    on the trajectory.
    """

    def __init__(self):
        super(MotionPlanning, self).__init__("MotionPlanning")
        self.role_name = self.get_param("role_name", "hero")
        self.control_loop_rate = self.get_param("control_loop_rate", 0.05)

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

        # Subscriber

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
        self.speed_limit_publisher: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/speed_limit", qos_profile=1
        )

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
                msg = f"Adjusted existing overtake. Used bigger offset: {req.offset}."
            else:
                msg = "Overwrote existing overtake."
            if (
                req.has_start_pos
                and self.overtake_status.status == OvertakeStatusResponse.OVERTAKING
            ):
                # We are already overtaking, continue to do so
                req.has_start_pos = False
                msg += " Ignored new start point, because overtake is already running."

        if self.overtake_status.status == OvertakeStatusResponse.NO_OVERTAKE:
            self.overtake_status.status = OvertakeStatusResponse.OVERTAKE_QUEUED

        self.overtake_status.offset = req.offset
        self.overtake_request = req

        rospy.loginfo(f"MotionPlanning: {msg}")
        return StartOvertakeResponse(success=True, msg=msg)

    def __end_overtake_callback(self, req: EndOvertakeRequest) -> EndOvertakeResponse:
        msg = "No overtake aborted: No overtake has been running"
        if self.overtake_request is not None:
            if req.has_end_pos:
                self.overtake_request.has_end_pos = True
                self.overtake_request.end_pos = req.end_pos
                self.overtake_request.end_transition_length = req.end_transition_length
                msg = "Adjusted end of existing overtake"
            else:
                self.overtake_request = None
                msg = "Cancelled existing overtake"
                if self.overtake_status.status == OvertakeStatusResponse.OVERTAKING:
                    self.overtake_status.status = OvertakeStatusResponse.OVERTAKE_ENDING
                else:
                    self.overtake_status.status = OvertakeStatusResponse.NO_OVERTAKE

        rospy.loginfo(f"MotionPlanning: {msg}")
        return EndOvertakeResponse(success=True, msg=msg)

    def __overtake_status_callback(
        self, req: OvertakeStatusRequest
    ) -> OvertakeStatusResponse:
        return self.overtake_status

    def __set_heading(self, data: Float32):
        """Set current Heading

        Args:
            data (Float32): Current Heading vom Subscriber
        """
        self.current_heading = data.data

    def __set_current_pos(self, data: PoseStamped):
        """Sets the current position and
        publishes a new trajectory_local based on it

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
            max_idx = None
            min_idx = 0
            self.init_trajectory = False
        else:
            self._update_current_global_waypoint_idx()
            max_length = 200.0
            max_idx = self.current_global_waypoint_idx + int(max_length * 2)
            min_idx = max(0, self.current_global_waypoint_idx - int(max_length * 2))

        global_traj_line = mapping_common.mask.ros_path_to_line(
            self.global_trajectory, start_idx=min_idx, end_idx=max_idx
        )
        global_traj_line = self._apply_overtake(
            global_traj_line, hero_transform=hero_transform
        )

        local_trajectory = mapping_common.mask.build_trajectory(
            global_traj_line,
            hero_transform,
            max_length=max_length,
            centered=False,
        )
        if local_trajectory is None:
            # Try to reinitialize trajectory on next position update
            self.init_trajectory = True
            rospy.logfatal_throttle(1.0, "MotionPlanning: Empty trajectory")
            return

        (start_x, start_y) = local_trajectory.coords[0]
        start_vector = Vector2.new(start_x, start_y)
        if start_vector.length() > 100.0:
            # We are far away from the trajectory
            # Try to reinitialize trajectory on next position update
            self.init_trajectory = True
            rospy.logfatal_throttle(1.0, "MotionPlanning: Too far away from trajectory")
            # Do not return here, otherwise the car does not continue driving

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
        if self.current_pos is None:
            return
        while len(self.global_trajectory.poses) > self.current_global_waypoint_idx + 1:
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
                # Check if we need to count the index backwards
                last_pose: Pose = self.global_trajectory.poses[
                    self.current_global_waypoint_idx - 1
                ].pose
                last_point = Point2.from_ros_msg(last_pose.position)
                if self.current_pos.distance_to(point0) > self.current_pos.distance_to(
                    last_point
                ):
                    self.current_global_waypoint_idx = max(
                        0, self.current_global_waypoint_idx - 1
                    )
                else:
                    break
            else:
                break

    def _apply_overtake(
        self, global_trajectory: LineString, hero_transform: Transform2D
    ) -> LineString:
        """Adds an overtake trajectory to *global_trajectory*
        if self.overtake_request is set.

        Args:
            global_trajectory (LineString)
            hero_transform (Transform2D)

        Returns:
            LineString: global_trajectory modified with an overtake
        """
        hero = mapping_common.hero.create_hero_entity()
        front_point: Point2 = hero_transform * Point2.new(hero.get_front_x(), 0.0)
        front_point_s = front_point.to_shapely()

        if self.overtake_request is None:
            distance_to_trajectory = shapely.distance(global_trajectory, front_point_s)
            if distance_to_trajectory < TRAJECTORY_DISTANCE_THRESHOLD:
                self.overtake_status.status = OvertakeStatusResponse.NO_OVERTAKE
            return global_trajectory

        hero_point = hero_transform.translation().point()

        before_trajectory: Optional[LineString] = None
        overtake_trajectory: Optional[LineString] = global_trajectory
        after_trajectory: Optional[LineString] = None

        overtake_request = self.overtake_request
        if not overtake_request.has_start_pos:
            overtake_request.has_start_pos = True
            overtake_request.start_pos = hero_point.to_ros_msg()

        global_start_point_s = Point2.from_ros_msg(
            overtake_request.start_pos
        ).to_shapely()
        start_dist = overtake_trajectory.line_locate_point(global_start_point_s)
        (before_trajectory, overtake_trajectory) = mapping_common.mask.split_line_at(
            overtake_trajectory, start_dist
        )

        if overtake_request.has_end_pos and overtake_trajectory is not None:
            global_end_point_s = Point2.from_ros_msg(
                overtake_request.end_pos
            ).to_shapely()
            end_dist = overtake_trajectory.line_locate_point(global_end_point_s)
            (overtake_trajectory, after_trajectory) = mapping_common.mask.split_line_at(
                overtake_trajectory, end_dist
            )

            if overtake_trajectory is None:
                # If we are after the end of the overtake -> delete overtake_request
                rospy.loginfo("MotionPlanning: Overtake ending")
                self.overtake_request = None
                self.overtake_status.status = OvertakeStatusResponse.OVERTAKE_ENDING

        if overtake_trajectory is None:
            return global_trajectory

        # Create the overtake by offsetting
        overtake_trajectory = overtake_trajectory.offset_curve(
            distance=overtake_request.offset
        )

        # We only start overtaking if we are
        # close enough to the overtake trajectory.
        # In local coordinated the position of the car is (0, 0), but
        # Using the front (hood) position for the check is better
        distance_to_overtake = shapely.distance(overtake_trajectory, front_point_s)
        if distance_to_overtake < TRAJECTORY_DISTANCE_THRESHOLD:
            self.overtake_status.status = OvertakeStatusResponse.OVERTAKING

        # Apply "smooth" transition by cropping the before and after parts
        ot_length = overtake_trajectory.length
        overtake_trajectory_cropped = mapping_common.mask.clamp_line(
            overtake_trajectory,
            start_distance=overtake_request.start_transition_length,
            end_distance=ot_length - overtake_request.end_transition_length,
        )
        if overtake_trajectory_cropped is not None:
            overtake_trajectory = overtake_trajectory_cropped

        # Build final trajectory by appending coordinates
        coords = np.array(overtake_trajectory.coords)
        if before_trajectory is not None:
            coords = np.append(np.array(before_trajectory.coords), coords, axis=0)
        if after_trajectory is not None:
            coords = np.append(coords, np.array(after_trajectory.coords), axis=0)

        overtake_trajectory = LineString(coords)
        return overtake_trajectory

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

import py_trees
from py_trees.common import Status
from typing import Optional

from rclpy.client import Client
from rclpy.publisher import Publisher
from rclpy.clock import Clock
from rclpy.duration import Duration

from std_msgs.msg import String, Bool, Float32
from geometry_msgs.msg import PoseStamped
from perception_interfaces.msg import Waypoint, TrafficLightState
from planning_interfaces.srv import OvertakeStatus
from carla_msgs.msg import CarlaRoute

import mapping_common.hero
from mapping_common.map import Map, MapTree
from mapping_common.entity import FlagFilter
from mapping_common.markers import debug_marker
from mapping_common.shape import Rectangle
from mapping_common.transform import Transform2D, Point2, Vector2
import shapely
from shapely.ops import nearest_points
from planning.behavior_agent.blackboard_utils import Blackboard

from . import behavior_names as bs
from .stop_mark_service_utils import (
    update_stop_marks,
)
from .overtake_service_utils import (
    request_overtake_status,
    request_end_overtake,
)
from .speed_alteration import add_speed_limit
from .waypoint_utils import calculate_waypoint_distance
from .debug_markers import add_debug_marker, add_debug_entry, debug_status
from .topics2blackboard import BLACKBOARD_MAP_ID
from . import get_logger

INTERSECTION_MARKER_COLOR = (222 / 255, 23 / 255, 214 / 255, 1.0)

"""
Source: https://github.com/ll7/psaf2
"""


def tr_status_str(t: Optional[TrafficLightState]):
    if t is None:
        return "UNKNOWN"
    if t.state == TrafficLightState.GREEN:
        return "GREEN"
    if t.state == TrafficLightState.YELLOW:
        return "YELLOW"
    if t.state == TrafficLightState.RED:
        return "RED"

    return "UNKNOWN"


INTERSECTION_LINE_STOPMARKS_ID = "intersection"
INTERSECTION_LEFT_STOPMARKS_ID = "intersection_left"

INTERSECTION_START_MIN_DISTANCE = 10.0
"""Distance at which we (force) enter into the approach behavior
if it was not possible before.

This distance is based roughly on the front of the car

Example: A running overtake delays the intersection behavior
"""

WAIT_TARGET_DISTANCE = 2.0
"""Target distance under which the car waits in front of the stop line

This distance is based roughly on the front of the car
"""

STOP_LINE_OFFSET = -1.0
"""Offset of the stop line compared to the position in the waypoint

Offset points in forward direction of the hero
"""

INTERSECTION_END_DISTANCE = 10.0
"""Distance after the intersection endpoint for leaving the behavior
"""

OVER_STOP_LINE_DIST_THRESHOLD = 0.2
"""Distance to the stop line at which we drove over it
"""

# Global variables which are updated by the behaviors below:

CURRENT_INTERSECTION_WAYPOINT: Optional[Waypoint]
"""Waypoint of the current intersection
"""

INTERSECTION_HAS_TRAFFIC_LIGHT: bool = False
"""True, if the car saw a traffic light in the intersection
at least once
"""

CURRENT_PRIORITY_CHECK_REFERENCE: Optional[Transform2D] = None
"""Global pose of the hero when the priority-cross-traffic check started."""

# Cross traffic parameter
CROSS_TRAFFIC_SPEED_THRESHOLD = 2.5  # m/s
CROSS_CHECK_DISTANCE = 15.0
CROSS_CHECK_LENGTH = 25.0
CROSS_CHECK_WIDTH = 50.0

# Priority traffic parameter
PRIORITY_SPEED_THRESHOLD = 25.0 / 3.6  # m/s ≈ 6.94
PRIORITY_CHECK_DISTANCE = 13.0  # further ahead in the direction of travel
PRIORITY_CHECK_LENGTH = 25.0
PRIORITY_CHECK_WIDTH = 50.0
PRIORITY_CLOSING_SPEED_THRESHOLD = 0.5
PRIORITY_CONFLICT_MARGIN = 1.5
PRIORITY_CONFLICT_CORE_LENGTH = 15.0
PRIORITY_TTC_THRESHOLD = 1.5
PRIORITY_TTC_EGO_SPEED_FLOOR = 3.0
PRIORITY_TTC_GATE_MIN_EGO_SPEED = 1.0
PRIORITY_UNSAFE_HOLD_TIME = 0.2
PRIORITY_SAFE_HOLD_TIME = 0.6
PRIORITY_PASS_JUDGE_DECELERATION = 3.0
PRIORITY_PASS_JUDGE_RESPONSE_TIME = 0.5
PRIORITY_PASS_JUDGE_MARGIN = 1.0
SELF_EMERGENCY_THRESHOLD = 10 / 3.6  # m/s ≈ 2.78


def _dot(a: Vector2, b: Vector2) -> float:
    return a.x() * b.x() + a.y() * b.y()


def _pose_to_transform(
    position: Optional[PoseStamped], heading: Optional[Float32]
) -> Optional[Transform2D]:
    if position is None or heading is None:
        return None
    return Transform2D.new_rotation_translation(
        heading.data,
        Vector2.new(position.pose.position.x, position.pose.position.y),
    )


def _get_current_pose_transform(blackboard: Blackboard) -> Optional[Transform2D]:
    current_pos: Optional[PoseStamped] = blackboard.try_get(
        "/paf/hero/global_current_pos"
    )
    current_heading: Optional[Float32] = blackboard.try_get(
        "/paf/hero/global_current_heading"
    )
    return _pose_to_transform(current_pos, current_heading)


def _build_priority_check_mask(
    hero,
    reference_pose: Optional[Transform2D] = None,
    current_pose: Optional[Transform2D] = None,
):
    offset = Transform2D.new_translation(
        Vector2.new(PRIORITY_CHECK_DISTANCE + hero.get_front_x(), 0.0)
    )
    rect = Rectangle(
        length=PRIORITY_CHECK_LENGTH,
        width=PRIORITY_CHECK_WIDTH,
        offset=offset,
    )
    target_point = Point2.from_vector(offset.translation())

    if reference_pose is not None and current_pose is not None:
        mask_transform = current_pose.inverse() * reference_pose
    else:
        mask_transform = hero.transform

    return rect.to_shapely(mask_transform), mask_transform * target_point


def _build_priority_check_centerline(
    hero,
    reference_pose: Optional[Transform2D] = None,
    current_pose: Optional[Transform2D] = None,
):
    offset_x = PRIORITY_CHECK_DISTANCE + hero.get_front_x()
    half_length = PRIORITY_CONFLICT_CORE_LENGTH / 2.0
    start_point = Point2.new(offset_x - half_length, 0.0)
    end_point = Point2.new(offset_x + half_length, 0.0)

    if reference_pose is not None and current_pose is not None:
        mask_transform = current_pose.inverse() * reference_pose
    else:
        mask_transform = hero.transform

    start_point = mask_transform * start_point
    end_point = mask_transform * end_point
    return shapely.LineString([start_point.to_shapely(), end_point.to_shapely()])


def _get_entity_velocity_in_hero_frame(entity) -> Optional[Vector2]:
    if entity.motion is None:
        return None
    return entity.transform * entity.motion.linear_motion


def _is_priority_cross_traffic_threat(
    entity, target_point: Point2, hero_width: float
) -> bool:
    return (
        _get_priority_conflict_point(
            entity,
            target_point,
            shapely.LineString([[0.0, 0.0], [target_point.x(), target_point.y()]]),
            hero_width,
        )
        is not None
    )


def _get_priority_conflict_point(
    entity,
    target_point: Point2,
    priority_centerline: shapely.LineString,
    hero_width: float,
) -> Optional[Point2]:
    velocity = _get_entity_velocity_in_hero_frame(entity)
    if velocity is None or velocity.length() <= PRIORITY_SPEED_THRESHOLD:
        return None

    entity_position = Point2.from_vector(entity.transform.translation())
    to_target = entity_position.vector_to(target_point)
    if to_target.length() == 0.0:
        return target_point

    direction = velocity.normalized()
    closing_speed = _dot(velocity, to_target.normalized())
    if closing_speed <= PRIORITY_CLOSING_SPEED_THRESHOLD:
        return None

    along_path = _dot(direction, to_target)
    if along_path <= 0.0:
        return None

    conflict_radius = (
        PRIORITY_CONFLICT_MARGIN + (entity.get_width() * 0.5) + (hero_width * 0.5)
    )
    entity_path_end = entity_position + (
        direction * max(along_path + PRIORITY_CHECK_LENGTH, PRIORITY_CHECK_LENGTH)
    )
    entity_path = shapely.LineString(
        [entity_position.to_shapely(), entity_path_end.to_shapely()]
    )
    hero_point, entity_point = nearest_points(
        priority_centerline,
        entity_path,
    )
    if hero_point.distance(entity_point) >= conflict_radius:
        return None

    return Point2.new(hero_point.x, hero_point.y)


def _is_priority_ttc_conflict(
    entity,
    target_point: Point2,
    ego_speed: float,
) -> bool:
    velocity = _get_entity_velocity_in_hero_frame(entity)
    if velocity is None or ego_speed < PRIORITY_TTC_GATE_MIN_EGO_SPEED:
        return True

    entity_position = Point2.from_vector(entity.transform.translation())
    to_target = entity_position.vector_to(target_point)
    along_path = _dot(velocity.normalized(), to_target)
    if along_path <= 0.0:
        return False

    entity_ttc = along_path / velocity.length()
    ego_ttc = target_point.vector().length() / max(
        ego_speed, PRIORITY_TTC_EGO_SPEED_FLOOR
    )
    return abs(entity_ttc - ego_ttc) <= PRIORITY_TTC_THRESHOLD


def _apply_priority_decision_hysteresis(
    raw_clear: bool, filtered_clear: bool, state_age_seconds: float
) -> bool:
    if raw_clear == filtered_clear:
        return filtered_clear

    if raw_clear:
        return state_age_seconds >= PRIORITY_SAFE_HOLD_TIME

    return not (state_age_seconds >= PRIORITY_UNSAFE_HOLD_TIME)


def _update_priority_decision_state(behavior, raw_clear: bool) -> bool:
    now = behavior.clock.now()
    if behavior.priority_raw_clear != raw_clear:
        behavior.priority_raw_clear = raw_clear
        behavior.priority_raw_state_since = now

    state_age_seconds = (
        now - behavior.priority_raw_state_since
    ).nanoseconds / 1_000_000_000
    behavior.priority_filtered_clear = _apply_priority_decision_hysteresis(
        raw_clear,
        behavior.priority_filtered_clear,
        state_age_seconds,
    )
    return behavior.priority_filtered_clear


def _is_over_priority_pass_judge_line(
    distance_to_conflict: float, ego_speed: float
) -> bool:
    ego_speed = max(ego_speed, 0.0)
    stopping_distance = (
        (ego_speed * ego_speed) / (2.0 * PRIORITY_PASS_JUDGE_DECELERATION)
        + ego_speed * PRIORITY_PASS_JUDGE_RESPONSE_TIME
        + PRIORITY_PASS_JUDGE_MARGIN
    )
    return distance_to_conflict <= stopping_distance


def _find_priority_cross_traffic_threat(
    map: Map,
    tree: MapTree,
    reference_pose: Optional[Transform2D] = None,
    current_pose: Optional[Transform2D] = None,
):
    hero = map.hero()
    if hero is None:
        return None, None, None
    hero_width = hero.get_width()

    mask, target_point = _build_priority_check_mask(
        hero,
        reference_pose=reference_pose,
        current_pose=current_pose,
    )
    priority_centerline = _build_priority_check_centerline(
        hero,
        reference_pose=reference_pose,
        current_pose=current_pose,
    )

    shapely_entities = tree.get_overlapping_entities(mask)

    for se in shapely_entities:
        entity = se.entity
        conflict_point = _get_priority_conflict_point(
            entity,
            target_point,
            priority_centerline,
            hero_width,
        )
        if conflict_point is not None:
            return entity, mask, conflict_point

    return None, mask, target_point


def check_priority_cross_traffic(
    map: Map,
    tree: MapTree,
    reference_pose: Optional[Transform2D] = None,
    current_pose: Optional[Transform2D] = None,
):
    """
    Checks whether there are fast vehicles in the larger intersection area.

    Returns:
        (priority_clear, mask_polygon)
        priority_clear = True  → no fast cross traffic detected
        priority_clear = False → fast cross traffic detected
    """
    threat_entity, mask, _ = _find_priority_cross_traffic_threat(
        map,
        tree,
        reference_pose=reference_pose,
        current_pose=current_pose,
    )

    return threat_entity is None, mask


def set_line_stop(client: Client, distance: float):
    """Sets the stop line at distance from the front of the hero

    Args:
        client (Client)
        distance (float): distance from the front of the hero
    """
    hero = mapping_common.hero.create_hero_entity()
    transform = Transform2D.new_translation(
        Vector2.new(distance + hero.get_front_x(), 0.0)
    )
    mask = Rectangle(0.5, 10.0, offset=transform)
    update_stop_marks(
        client,
        id=INTERSECTION_LINE_STOPMARKS_ID,
        reason="intersection stop",
        is_global=False,
        marks=[mask],
    )


def unset_line_stop(client: Client):
    update_stop_marks(
        client,
        id=INTERSECTION_LINE_STOPMARKS_ID,
        reason="intersection clear",
        is_global=False,
        marks=[],
    )


def set_left_stop(client: Client):
    # Just an empty map
    map = Map()
    # We just use the lane free function to create the shape for our stopmarker
    tree = map.build_tree()
    _, mask = tree.is_lane_free(
        False,
        lane_length=35.0,
        lane_transform=2.0,
        reduce_lane=0.75,
        check_method="rectangle",
    )
    if isinstance(mask, shapely.Polygon):
        update_stop_marks(
            client,
            id=INTERSECTION_LEFT_STOPMARKS_ID,
            reason="intersection left stop",
            is_global=False,
            marks=[mask],
        )


def unset_left_stop(client: Client):
    update_stop_marks(
        client,
        id=INTERSECTION_LEFT_STOPMARKS_ID,
        reason="intersection left clear",
        is_global=False,
        marks=[],
    )


def apply_emergency_vehicle_speed_fix():
    if (
        CURRENT_INTERSECTION_WAYPOINT is not None
        and CURRENT_INTERSECTION_WAYPOINT.road_option == CarlaRoute.STRAIGHT
        and INTERSECTION_HAS_TRAFFIC_LIGHT
    ):
        # We drive slower in straight intersections with traffic light
        # to avoid emergency vehicles
        add_speed_limit(3.0)


class Ahead(py_trees.behaviour.Behaviour):
    """
    This behaviour checks whether there is an intersection in front of the
    ego vehicle or not and triggers the rest of the decision tree handling the
     intersection.
    """

    def __init__(self, name: str, stop_client: Client, overtake_status_client: Client):
        super().__init__(name)
        self.stop_client = stop_client
        self.overtake_status_client = overtake_status_client

    def setup(self, **kwargs):
        self.blackboard = Blackboard()

    def initialise(self):
        global INTERSECTION_HAS_TRAFFIC_LIGHT, CURRENT_PRIORITY_CHECK_REFERENCE
        INTERSECTION_HAS_TRAFFIC_LIGHT = False
        CURRENT_PRIORITY_CHECK_REFERENCE = None

    def update(self):
        """
        :return: py_trees.common.Status.SUCCESS, if the vehicle is within range
                    of the intersection
                 py_trees.common.Status.FAILURE, if we are too far away from
                 the intersection
        """
        waypoint: Optional[Waypoint] = self.blackboard.try_get(
            "/paf/hero/current_waypoint"
        )
        if waypoint is None:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "No waypoint"
            )

        if not waypoint.waypoint_type == Waypoint.TYPE_INTERSECTION:
            return py_trees.common.Status.FAILURE
        global CURRENT_INTERSECTION_WAYPOINT
        CURRENT_INTERSECTION_WAYPOINT = waypoint

        stop_line_distance = calculate_waypoint_distance(
            self.blackboard,
            CURRENT_INTERSECTION_WAYPOINT,
            forward_offset=STOP_LINE_OFFSET,
        )
        if stop_line_distance is None:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Missing information for stop_line distance calculation",
            )

        add_debug_entry(self.name, f"Stop line distance: {stop_line_distance}")

        if (
            stop_line_distance < 100
            and stop_line_distance > OVER_STOP_LINE_DIST_THRESHOLD
        ):
            # Add the stop line early to avoid running over it at high speeds
            set_line_stop(self.stop_client, stop_line_distance)

        if stop_line_distance > 25:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Stop line too far away"
            )
        elif stop_line_distance <= OVER_STOP_LINE_DIST_THRESHOLD:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Already over stop line"
            )

        overtake_status = request_overtake_status(self.overtake_status_client)
        if overtake_status is None:
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                "Unable to get overtake status",
            )

        # Stay in the overtake behavior as long as we can
        if stop_line_distance > INTERSECTION_START_MIN_DISTANCE and (
            overtake_status.status == OvertakeStatus.Response.OVERTAKE_QUEUED
            or overtake_status.status == OvertakeStatus.Response.OVERTAKING
        ):
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                f"Waiting until {INTERSECTION_START_MIN_DISTANCE}m before intersection "
                "for overtake to finish",
            )

        return debug_status(self.name, py_trees.common.Status.SUCCESS)

    def terminate(self, new_status):
        if new_status is Status.INVALID:
            unset_line_stop(self.stop_client)


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an intersection and intersection_ahead is
    triggered. It than handles the approaching the intersection, slowing the
    vehicle down appropriately.
    """

    def __init__(
        self,
        name: str,
        curr_behavior_pub: Publisher,
        stop_client: Client,
        end_overtake_client: Client,
    ):
        super().__init__(name)
        self.curr_behavior_pub = curr_behavior_pub
        self.stop_client = stop_client
        self.end_overtake_client = end_overtake_client

    def setup(self, **kwargs):
        self.blackboard = Blackboard()

    def initialise(self):
        """
        This initializes the variables needed to save information about the
        stop line, stop signs and the traffic light.
        """
        get_logger().info("Approaching Intersection")
        self.curr_behavior_pub.publish(String(data=bs.int_app_init.name))
        request_end_overtake(self.end_overtake_client)

    def update(self):
        """
        Gets the current traffic light status, stop sign status
        and the stop line distance. Calcualtes a virtual stop line and
        publishes a distance to it. Slows down car until virtual stop line
        is reached when there is a red traffic light or a stop sign.
        :return: py_trees.common.Status.RUNNING, if too far from intersection
                 py_trees.common.Status.SUCCESS, if stopped in front of inter-
                 section or entered the intersection
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        global CURRENT_INTERSECTION_WAYPOINT
        if CURRENT_INTERSECTION_WAYPOINT is None:
            get_logger().error(
                "Intersection behavior: CURRENT_INTERSECTION_WAYPOINT not set"
            )
            return debug_status(
                self.name,
                Status.FAILURE,
                "Error: CURRENT_INTERSECTION_WAYPOINT not set",
            )

        # Update Light Info
        traffic_light_status: Optional[TrafficLightState] = self.blackboard.try_get(
            "/paf/hero/Center/traffic_light_state"
        )
        traffic_light_detected: bool = False
        if traffic_light_status is not None:
            traffic_light_detected = (
                traffic_light_status.state != TrafficLightState.UNKNOWN
            )
        else:
            traffic_light_status = TrafficLightState(state=TrafficLightState.UNKNOWN)
        if traffic_light_detected:
            global INTERSECTION_HAS_TRAFFIC_LIGHT
            INTERSECTION_HAS_TRAFFIC_LIGHT = True

        add_debug_entry(
            self.name, f"Traffic light: {tr_status_str(traffic_light_status)}"
        )
        add_debug_entry(self.name, f"Traffic light detected: {traffic_light_detected}")

        stop_line_distance = calculate_waypoint_distance(
            self.blackboard,
            CURRENT_INTERSECTION_WAYPOINT,
            forward_offset=STOP_LINE_OFFSET,
        )
        if stop_line_distance is None:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Missing information for stop_line distance calculation",
            )
        add_debug_entry(self.name, f"Stop line distance: {stop_line_distance}")
        if stop_line_distance <= OVER_STOP_LINE_DIST_THRESHOLD:
            # We already drove over the stopline...go to next behavior
            return debug_status(
                self.name,
                Status.SUCCESS,
                "Drove over stopline",
            )

        # stop when there is no or red/yellow traffic light
        if (
            not traffic_light_detected
            or traffic_light_status.state == TrafficLightState.YELLOW
            or traffic_light_status.state == TrafficLightState.RED
        ):
            self.curr_behavior_pub.publish(String(data=bs.int_app_to_stop.name))
            set_line_stop(self.stop_client, stop_line_distance)

            # We are stopping: check if we are standing < WAIT_TARGET_DISTANCE
            # in front of the line

            # get speed
            speedometer = self.blackboard.try_get("/carla/hero/Speed")
            if speedometer is None:
                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    "No speedometer connected",
                )
            speed: float = speedometer.speed

            if speed < 0.1 and stop_line_distance < WAIT_TARGET_DISTANCE:
                return debug_status(
                    self.name, py_trees.common.Status.SUCCESS, "Stopped at stop_line"
                )
        else:
            # Green light
            self.curr_behavior_pub.publish(String(data=bs.int_app_green.name))
            unset_line_stop(self.stop_client)
            if stop_line_distance < WAIT_TARGET_DISTANCE:
                return debug_status(
                    self.name, py_trees.common.Status.SUCCESS, "Driving over stop_line"
                )

        if CURRENT_INTERSECTION_WAYPOINT.road_option == CarlaRoute.LEFT:
            set_left_stop(self.stop_client)

        return debug_status(
            self.name,
            py_trees.common.Status.RUNNING,
            "Still approaching intersection...",
        )

    def terminate(self, new_status):
        if new_status is Status.FAILURE or new_status is Status.INVALID:
            unset_line_stop(self.stop_client)


class Wait(py_trees.behaviour.Behaviour):
    """
    This behavior handles the waiting in front of the stop line at the inter-
    section until the vehicle is allowed to drive through.
    """

    def __init__(
        self,
        name: str,
        clock: Clock,
        curr_behavior_pub: Publisher,
        stop_client: Client,
        emergency_pub: Publisher,
    ):
        super().__init__(name)
        self.clock = clock
        self.curr_behavior_pub = curr_behavior_pub
        self.stop_client = stop_client
        self.emergency_pub = emergency_pub

    def setup(self, **kwargs):
        self.blackboard = Blackboard()
        self.green_light_time = None

    def initialise(self):
        get_logger().info("Wait Intersection")
        global CURRENT_INTERSECTION_WAYPOINT
        if CURRENT_INTERSECTION_WAYPOINT is None:
            get_logger().error(
                "Intersection behavior: CURRENT_INTERSECTION_WAYPOINT not set"
            )
            return
        self.waypoint = CURRENT_INTERSECTION_WAYPOINT

        self.green_light_time = self.clock.now()
        self.over_stop_line = False
        self.oncoming_counter = 0
        self.stop_time = self.clock.now()
        self.was_red = False
        self.intersection_type = self.waypoint.road_option
        self.left_marker_set = False
        self.priority_raw_clear = True
        self.priority_filtered_clear = True
        self.priority_raw_state_since = self.clock.now()

    def update(self):
        """
        Waits in front of the intersection until there is a green light.
        In case of turning left, oncoming traffic is checked bevor proceeding.
        :return: py_trees.common.Status.RUNNING, while traffic light is yellow
                 or red or oncoming is blocked.
                 py_trees.common.Status.SUCCESS, if the traffic light switched
                 to green or no traffic light is detected and oncoming is free
                 when turning left.
        """
        self.curr_behavior_pub.publish(String(data=bs.int_wait.name))
        map: Optional[Map] = self.blackboard.try_get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Map is None"
            )
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        if self.intersection_type != CarlaRoute.LEFT:
            global CURRENT_PRIORITY_CHECK_REFERENCE
            current_pose = _get_current_pose_transform(self.blackboard)
            if CURRENT_PRIORITY_CHECK_REFERENCE is None:
                CURRENT_PRIORITY_CHECK_REFERENCE = current_pose

            threat_entity, priority_mask, _ = _find_priority_cross_traffic_threat(
                map,
                tree,
                reference_pose=CURRENT_PRIORITY_CHECK_REFERENCE,
                current_pose=current_pose,
            )
            raw_priority_clear = threat_entity is None
            priority_clear = _update_priority_decision_state(self, raw_priority_clear)
            add_debug_entry(
                self.name,
                f"[Wait] Priority cross traffic clear: {priority_clear}"
                f" (raw={raw_priority_clear})",
            )
            if priority_mask is not None:
                add_debug_marker(
                    debug_marker(priority_mask, color=(1.0, 0.0, 0.0, 0.3))
                )

            if not priority_clear:
                # priority cross traffic detected
                set_line_stop(self.stop_client, 0.0)

                speedometer = self.blackboard.try_get("/carla/hero/Speed")
                ego_speed = speedometer.speed if speedometer is not None else 0.0

                if ego_speed > SELF_EMERGENCY_THRESHOLD:
                    self.emergency_pub.publish(Bool(data=True))
                    reason = (
                        f"WAIT: EMERGENCY - fast cross traffic, "
                        f"ego_speed={ego_speed:.2f} m/s"
                    )
                else:
                    reason = (
                        f"WAIT: fast cross traffic, but ego_speed={ego_speed:.2f} m/s "
                        "(no emergency brake)"
                    )

                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    reason,
                )
            unset_line_stop(self.stop_client)
            self.emergency_pub.publish(Bool(data=False))

        dist = calculate_waypoint_distance(
            self.blackboard, self.waypoint, forward_offset=STOP_LINE_OFFSET
        )
        if dist is None:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Missing information for stop_line distance calculation",
            )

        traffic_light_status = self.blackboard.try_get(
            "/paf/hero/Center/traffic_light_state"
        )
        traffic_light_detected: bool = False
        if traffic_light_status is not None:
            traffic_light_detected = (
                traffic_light_status.state != TrafficLightState.UNKNOWN
            )
        else:
            traffic_light_status = TrafficLightState(state=TrafficLightState.UNKNOWN)
        if traffic_light_detected:
            global INTERSECTION_HAS_TRAFFIC_LIGHT
            INTERSECTION_HAS_TRAFFIC_LIGHT = True

        add_debug_entry(
            self.name, f"Traffic light status: {tr_status_str(traffic_light_status)}"
        )
        add_debug_entry(self.name, f"Intersection type: {self.intersection_type}")

        if self.intersection_type == CarlaRoute.LEFT and not self.left_marker_set:
            set_left_stop(self.stop_client)
            self.left_marker_set = True

        # First check if we still need to wait at the stop line
        if self.over_stop_line is False:
            set_line_stop(self.stop_client, dist)
            if dist <= OVER_STOP_LINE_DIST_THRESHOLD:
                self.over_stop_line = True
            elif (
                traffic_light_status.state == TrafficLightState.RED
                or traffic_light_status.state == TrafficLightState.YELLOW
            ):
                # Wait at traffic light
                self.green_light_time = self.clock.now()
                self.curr_behavior_pub.publish(String(data=bs.int_wait.name))
                self.was_red = True
                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    "Waiting for traffic light",
                )
            elif traffic_light_status.state == TrafficLightState.UNKNOWN:
                # Wait at least 0.5 seconds at stopline (was at 2 sec)
                if self.clock.now() - self.stop_time > Duration(seconds=0.5):
                    self.over_stop_line = True
                    unset_line_stop(self.stop_client)
                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    "Waiting at stopline",
                )
            elif (
                self.clock.now() - self.green_light_time < Duration(seconds=1.0)
                and traffic_light_status.state == TrafficLightState.GREEN
                and self.was_red
            ):
                # Wait approx 1s for confirmation
                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    "Wait Confirm green light!",
                )
            elif (
                self.clock.now() - self.green_light_time > Duration(seconds=0.5)
                and traffic_light_status.state == TrafficLightState.GREEN
                and self.was_red
            ):
                # Drive through intersection
                self.over_stop_line = True
                unset_line_stop(self.stop_client)
                return debug_status(
                    self.name, py_trees.common.Status.RUNNING, "Driving through..."
                )
            else:
                # Light was green when switching to wait, drive through intersection
                self.over_stop_line = True

        unset_line_stop(self.stop_client)
        if self.intersection_type != CarlaRoute.LEFT:
            if self.over_stop_line:
                return debug_status(
                    self.name,
                    py_trees.common.Status.SUCCESS,
                    "No left turn -> continue",
                )
            return debug_status(
                self.name, py_trees.common.Status.RUNNING, "Driving over stop line"
            )

        self.curr_behavior_pub.publish(String(data=bs.int_wait.name))
        intersection_clear, intersection_mask = tree.is_lane_free_intersection(
            lane_length=self.blackboard.get("/params/left_check_length"),
            lane_transform_x=self.blackboard.get("/params/left_check_x_transform"),
        )
        add_debug_entry(self.name, f"Oncoming counter: {self.oncoming_counter}")
        add_debug_entry(self.name, f"Intersection clear: {intersection_clear}")
        add_debug_marker(
            debug_marker(intersection_mask, color=INTERSECTION_MARKER_COLOR)
        )
        if intersection_clear:
            self.oncoming_counter += 1
            if self.oncoming_counter > 2 and not self.blackboard.try_get(
                "/params/left_check_debug"
            ):
                self.curr_behavior_pub.publish(String(data=bs.int_enter.name))
                unset_left_stop(self.stop_client)
                return debug_status(
                    self.name, py_trees.common.Status.SUCCESS, "Intersection clear"
                )
            else:
                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    f"Intersection clear\n\tCounter wait: {self.oncoming_counter}",
                )
        else:
            self.oncoming_counter = 0
            return debug_status(
                self.name, py_trees.common.Status.RUNNING, "Intersection blocked"
            )

    def terminate(self, new_status):
        if new_status is Status.FAILURE or new_status is Status.INVALID:
            unset_line_stop(self.stop_client)
            unset_left_stop(self.stop_client)


class Enter(py_trees.behaviour.Behaviour):
    """
    This behavior handles the driving through an intersection, finishes
    after a certain distance threshold and ends the intersection behavior.
    """

    def __init__(
        self,
        name: str,
        clock: Clock,
        curr_behavior_pub: Publisher,
        stop_client: Client,
        emergency_pub: Publisher,
    ):
        super().__init__(name)
        self.clock = clock
        self.curr_behavior_pub = curr_behavior_pub
        self.stop_client = stop_client
        self.emergency_pub = emergency_pub

    def setup(self, **kwargs):
        self.blackboard = Blackboard()

    def initialise(self):
        get_logger().info("Enter Intersection")
        global CURRENT_INTERSECTION_WAYPOINT, CURRENT_PRIORITY_CHECK_REFERENCE
        if CURRENT_INTERSECTION_WAYPOINT is None:
            get_logger().error(
                "Intersection behavior: CURRENT_INTERSECTION_WAYPOINT not set"
            )
            return
        self.waypoint = CURRENT_INTERSECTION_WAYPOINT
        unset_line_stop(self.stop_client)
        self.curr_behavior_pub.publish(String(data=bs.int_enter.name))
        self.intersection_type = self.waypoint.road_option
        if CURRENT_PRIORITY_CHECK_REFERENCE is None:
            CURRENT_PRIORITY_CHECK_REFERENCE = _get_current_pose_transform(
                self.blackboard
            )
        self.priority_raw_clear = True
        self.priority_filtered_clear = True
        self.priority_raw_state_since = self.clock.now()

    def update(self):
        """
        Continues driving through the intersection until it is far way enough
        from CURRENT_INTERSECTION_WAYPOINT
        :return: py_trees.common.Status.RUNNING, if too far from the end of
                 the intersection
                 py_trees.common.Status.FAILURE, once finished.
        """
        global CURRENT_INTERSECTION_WAYPOINT
        if CURRENT_INTERSECTION_WAYPOINT is None:
            get_logger().error(
                "Intersection behavior: CURRENT_INTERSECTION_WAYPOINT not set"
            )
            return debug_status(
                self.name,
                Status.FAILURE,
                "Error: CURRENT_INTERSECTION_WAYPOINT not set",
            )

        # priority cross traffic check
        map: Optional[Map] = self.blackboard.try_get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(self.name, Status.FAILURE, "Map is None (Enter)")

        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        global CURRENT_PRIORITY_CHECK_REFERENCE
        current_pose = _get_current_pose_transform(self.blackboard)
        if CURRENT_PRIORITY_CHECK_REFERENCE is None:
            CURRENT_PRIORITY_CHECK_REFERENCE = current_pose

        speedometer = self.blackboard.try_get("/carla/hero/Speed")
        ego_speed = speedometer.speed if speedometer is not None else 0.0
        threat_entity, priority_mask, conflict_point = (
            _find_priority_cross_traffic_threat(
                map,
                tree,
                reference_pose=CURRENT_PRIORITY_CHECK_REFERENCE,
                current_pose=current_pose,
            )
        )
        pass_judge_distance = (
            conflict_point.vector().length() if conflict_point is not None else None
        )
        raw_priority_clear = threat_entity is None
        if not raw_priority_clear and conflict_point is not None:
            raw_priority_clear = not _is_priority_ttc_conflict(
                threat_entity,
                conflict_point,
                ego_speed,
            )

        priority_clear = _update_priority_decision_state(self, raw_priority_clear)
        add_debug_entry(
            self.name,
            f"[Enter] Priority cross traffic clear: {priority_clear}"
            f" (raw={raw_priority_clear})",
        )
        if priority_mask is not None:
            add_debug_marker(debug_marker(priority_mask, color=(1.0, 0.0, 0.0, 0.3)))

        if not priority_clear:
            if pass_judge_distance is not None and _is_over_priority_pass_judge_line(
                pass_judge_distance, ego_speed
            ):
                self.priority_raw_clear = True
                self.priority_filtered_clear = True
                self.priority_raw_state_since = self.clock.now()
                add_debug_entry(
                    self.name,
                    "[Enter] Over priority pass judge line, ignore new stop",
                )
            else:
                # priority cross traffic detected
                self.curr_behavior_pub.publish(String(data=bs.int_wait.name))
                set_line_stop(self.stop_client, 0.0)

                if ego_speed > SELF_EMERGENCY_THRESHOLD:
                    self.emergency_pub.publish(Bool(data=True))
                    reason = (
                        "ENTER: EMERGENCY – fast cross traffic, "
                        f"ego_speed={ego_speed:.2f} m/s"
                    )

                else:
                    reason = (
                        f"ENTER: fast cross traffic, but ego_speed={ego_speed:.2f} m/s "
                        "(no emergency brake)"
                    )

                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    reason,
                )
        unset_line_stop(self.stop_client)
        self.emergency_pub.publish(Bool(data=False))

        intersection_end_distance = calculate_waypoint_distance(
            self.blackboard, CURRENT_INTERSECTION_WAYPOINT, forward_offset=20
        )
        if intersection_end_distance is None:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Missing information for intersection_end_distance calculation",
            )
        add_debug_entry(
            self.name, f"Intersection end distance: {intersection_end_distance}"
        )

        # Distance does usually not reach exacty zero, use some margin
        if intersection_end_distance <= 0.5:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Left intersection"
            )

        self.curr_behavior_pub.publish(String(data=bs.int_enter.name))
        return debug_status(
            self.name, py_trees.common.Status.RUNNING, "Driving through..."
        )

    def terminate(self, new_status):
        global CURRENT_PRIORITY_CHECK_REFERENCE
        CURRENT_PRIORITY_CHECK_REFERENCE = None

import py_trees
from py_trees.common import Status
from typing import Optional
import shapely

from rclpy.client import Client
from rclpy.publisher import Publisher

from carla_msgs.msg import CarlaRoute
from geometry_msgs.msg import Point
from planning_interfaces.srv import OvertakeStatus
from perception_interfaces.msg import Waypoint

import mapping_common.mask
from mapping_common.map import Map, LaneFreeState, LaneFreeDirection
from mapping_common.entity import FlagFilter
from mapping_common.transform import Point2, Transform2D, Vector2
from mapping_common.markers import debug_marker
from mapping_common.shape import Rectangle

from . import behavior_names as bs
from .topics2blackboard import BLACKBOARD_MAP_ID
from .debug_markers import add_debug_marker, debug_status, add_debug_entry
from .overtake import OVERTAKE_SPACE_STOPMARKS_ID
from .overtake_service_utils import (
    request_start_overtake,
    request_end_overtake,
    request_overtake_status,
    get_global_hero_transform,
)
from .stop_mark_service_utils import (
    update_stop_marks,
)
from . import get_logger

from local_planner.utils import (
    TARGET_DISTANCE_TO_STOP_LANECHANGE,
    TARGET_DISTANCE_TO_TRIGGER_LANECHANGE,
)

LANECHANGE_MARKER_COLOR = (153 / 255, 50 / 255, 204 / 255, 1.0)
LANECHANGE_FREE = False
LANECHANGE_STOPMARK_ID = "lanechange"

"""
Source: https://github.com/ll7/psaf2
"""


class Ahead(py_trees.behaviour.Behaviour):
    """
    This behaviour checks whether there is a lane change in front of the
    ego vehicle or not and triggers the rest of the decision tree handling the
    lane change.
    """

    def __init__(
        self,
        name: str,
        overtake_status_client: Client,
        end_overtake_client: Client,
        stop_client: Client,
    ):
        super().__init__(name)
        self.overtake_status_client = overtake_status_client
        self.end_overtake_client = end_overtake_client
        self.stop_client = stop_client

    def setup(self, **kwargs):
        self.blackboard = py_trees.blackboard.Blackboard()

    def initialise(self):
        global LANECHANGE_FREE
        LANECHANGE_FREE = False
        self.change_detected = False
        self.change_distance: Optional[float] = None
        self.change_option: Optional[int] = None
        self.change_position: Optional[Point] = None

    def update(self):
        """
        Gets the current distance to the next lane change.
        :return: py_trees.common.Status.SUCCESS, if the vehicle is within range
                 of the lane change
                 py_trees.common.Status.RUNNING, if overtake was ended, wait for it
                 py_trees.common.Status.FAILURE, if we are too far away from
                 the lane change
        """
        global LANECHANGE_FREE
        waypoint: Optional[Waypoint] = self.blackboard.get("/paf/hero/current_waypoint")
        trajectory_local = self.blackboard.get("/paf/hero/trajectory_local")
        if waypoint is None or trajectory_local is None:
            return debug_status(
                self.name, Status.FAILURE, "waypoint or trajectory_local is None"
            )
        else:
            self.change_detected = waypoint.waypoint_type == Waypoint.TYPE_LANECHANGE
            self.change_option = waypoint.road_option
            self.change_position = waypoint.position
            # get change distance from global change point (transfered to local
            #  hero coords) as this is more accurate than lanechange msg distance
            hero_transform = get_global_hero_transform()
            local_pos: Point2 = hero_transform.inverse() * Point2.new(
                self.change_position.x, self.change_position.y
            )
            trajectory_local = mapping_common.mask.ros_path_to_line(trajectory_local)
            self.change_distance = trajectory_local.line_locate_point(
                local_pos.to_shapely()
            )

        if (
            self.change_distance is None
            or self.change_option is None
            or self.change_position is None
        ):
            return debug_status(
                self.name, Status.FAILURE, "At least one change parameter is None"
            )

        overtake_status = request_overtake_status(self.overtake_status_client)
        if overtake_status is None:
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                "Unable to get overtake status",
            )

        # multiplier for y coord, direction for left or right lane change
        lane_pos = 1
        if self.change_option == CarlaRoute.CHANGELANERIGHT:
            lane_pos = -1

        if (
            self.change_detected
            and self.change_distance < TARGET_DISTANCE_TO_TRIGGER_LANECHANGE
            and self.change_distance > 0
        ):
            # delete stop marks from overtake as they could block lane change
            update_stop_marks(
                self.stop_client,
                id=OVERTAKE_SPACE_STOPMARKS_ID,
                reason="lanechange delete overtake marks",
                is_global=False,
                marks=[],
            )
            # if overtake in process and lanechange is planned to left:
            # just end overtake on the left lane and lanechange is finished
            if (
                overtake_status == OvertakeStatus.Response.OVERTAKING
                or overtake_status == OvertakeStatus.Response.OVERTAKE_QUEUED
            ):
                if self.change_option == CarlaRoute.CHANGELANELEFT:
                    # change overtake end to the same lane if we are already in overtake
                    # and want a lanechange to left
                    end_transition_length = min(15.0, self.change_distance + 9.0)
                    request_start_overtake(
                        client=self.end_overtake_client,
                        local_end_pos=Point2.new(self.change_distance + 10.0, 0.0),
                        end_transition_length=end_transition_length,
                    )
                    LANECHANGE_FREE = True
                    return debug_status(
                        self.name,
                        Status.SUCCESS,
                        "Already in overtake, just adapted overtake end_pos. "
                        "no lane change",
                    )
                else:
                    request_end_overtake(self.end_overtake_client)
                    return debug_status(
                        self.name, Status.RUNNING, "Lane change ahead, aborted overtake"
                    )
            # if overtake queued: delete it,
            # if no overtake ahead: continue with lanechange
            else:
                # create local coords stop mark shape based
                # on global change position and current hero position
                stop_mark_shape = Rectangle(
                    length=25.0,
                    width=0.5,
                    offset=Transform2D.new_translation(
                        Vector2.new(local_pos.x() + 7.5, local_pos.y() + lane_pos * 1.5)
                    ),
                )
                update_stop_marks(
                    self.stop_client,
                    id=LANECHANGE_STOPMARK_ID,
                    reason="lane blocked",
                    is_global=False,
                    marks=[stop_mark_shape],
                )
                return debug_status(self.name, Status.SUCCESS, "Lane change ahead")

        else:
            return debug_status(
                self.name,
                Status.FAILURE,
                "No lane change ahead",
            )

    def terminate(self, new_status):
        pass


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an lane change and behaviours.road_features.LaneChangeAhead is
    triggered. It than handles approaching the lane change, slowing the
    vehicle down appropriately.
    """

    def __init__(
        self,
        name: str,
        curr_behavior_pub: Publisher,
        start_overtake_client: Client,
        stop_client: Client,
    ):
        super().__init__(name)
        self.curr_behavior_pub = curr_behavior_pub
        self.start_overtake_client = start_overtake_client
        self.stop_client = stop_client

    def setup(self, **kwargs):
        self.blackboard = py_trees.blackboard.Blackboard()

    def initialise(self):
        get_logger().info("Approaching Change")
        self.change_detected = False
        self.change_distance: Optional[float] = None
        self.change_option: Optional[int] = None
        self.change_direction: Optional[LaneFreeDirection] = None
        self.counter_lanefree = 0
        self.curr_behavior_pub.publish(bs.lc_app_init.name)

    def update(self):
        """
        Calculates a virtual stop line and slows down while approaching unless
        lane change is not blocked.
        :return: py_trees.common.Status.RUNNING, if too far from lane change
                 py_trees.common.Status.SUCCESS, if stopped in front of lane
                 change or entered the lane change
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        global LANECHANGE_FREE

        if LANECHANGE_FREE:
            self.curr_behavior_pub.publish(bs.lc_app_free.name)
            return debug_status(
                self.name,
                py_trees.common.Status.SUCCESS,
                "Lanechange free, skipping Approach and exit lane change behavior",
            )

        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(self.name, Status.FAILURE, "Lane Change: Map is None")
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        # Get lane change distance waypoint from blackboard
        waypoint: Optional[Waypoint] = self.blackboard.get("/paf/hero/current_waypoint")
        if waypoint is not None:
            self.change_distance = waypoint.distance
            self.change_detected = waypoint.waypoint_type == Waypoint.TYPE_LANECHANGE
            self.change_option = waypoint.road_option

            # Check if change is to the left or right lane
            if self.change_option == CarlaRoute.CHANGELANELEFT:
                self.change_direction = LaneFreeDirection.LEFT
            elif self.change_option == CarlaRoute.CHANGELANERIGHT:
                self.change_direction = LaneFreeDirection.RIGHT

        if (
            self.change_distance is None
            or self.change_option is None
            or self.change_direction is None
        ):
            return debug_status(
                self.name,
                Status.FAILURE,
                "Lane change: At least one change parameter is None",
            )

        add_debug_entry(
            self.name,
            f"Change distance: {self.change_distance}",
        )

        debug_entry_text = "None"
        if self.change_direction is not None:
            debug_entry_text = self.change_direction.name
        add_debug_entry(
            self.name,
            f"Change direction: {debug_entry_text}",
        )

        # if change to right, do not change early
        # (as there could be no road till change point!)
        if self.change_detected and self.change_direction is LaneFreeDirection.LEFT:
            lc_free, lc_mask = tree.is_lane_free(
                right_lane=self.change_direction.value,
                lane_length=22.5,
                lane_transform=-5.0,
                check_method="fallback",
            )
            if isinstance(lc_mask, shapely.Polygon):
                add_debug_marker(debug_marker(lc_mask, color=LANECHANGE_MARKER_COLOR))
            add_debug_entry(self.name, f"Lane change: Is lane free? {lc_free.name}")
            if lc_free is LaneFreeState.FREE:
                self.counter_lanefree += 1
                # using a counter to account for inconsistencies
                if self.counter_lanefree > 1:
                    # bool to skip Wait since oncoming is free
                    LANECHANGE_FREE = True
                    if self.change_direction is LaneFreeDirection.RIGHT:
                        lanechange_offset = -2.5
                    else:
                        lanechange_offset = 2.5
                    end_transition_length = min(15.0, self.change_distance + 9.0)
                    request_start_overtake(
                        client=self.start_overtake_client,
                        offset=lanechange_offset,
                        local_end_pos=Point2.new(
                            self.change_distance + 10.0, lanechange_offset
                        ),
                        end_transition_length=end_transition_length,
                    )
                    update_stop_marks(
                        self.stop_client,
                        id=LANECHANGE_STOPMARK_ID,
                        reason="lane not blocked",
                        is_global=False,
                        marks=[],
                    )
                    self.curr_behavior_pub.publish(bs.lc_app_free.name)
                    return debug_status(
                        self.name,
                        Status.SUCCESS,
                        "Lane Change: Lane free, changing directly",
                    )
                else:
                    self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
                    return debug_status(
                        self.name,
                        Status.RUNNING,
                        f"Lane Change: Free with count {self.counter_lanefree}/2",
                    )
            else:
                if lc_free is LaneFreeState.BLOCKED:
                    self.counter_lanefree = 0
                    add_debug_entry(
                        self.name,
                        "Lane Change: Lane blocked, reset count, stay in current lane",
                    )
                else:
                    add_debug_entry(
                        self.name,
                        "Lane Change: Lane free state unknown, "
                        f"keep count {self.counter_lanefree}/2, stay in current lane",
                    )
                self.curr_behavior_pub.publish(bs.lc_app_blocked.name)

        # currently TARGET_DISTANCE_TO_STOP_LANECHANGE == 5 (see utils.py)
        if self.change_distance <= TARGET_DISTANCE_TO_STOP_LANECHANGE:
            return debug_status(
                self.name,
                Status.SUCCESS,
                f"Lane Change: Reached target distance {self.change_distance}, "
                "stopping car and change to Wait",
            )
        else:
            if self.change_direction is LaneFreeDirection.LEFT:
                return debug_status(
                    self.name,
                    Status.RUNNING,
                    "Lane Change: Still approaching, stay in current lane",
                )
            return debug_status(
                self.name,
                Status.RUNNING,
                "Lane Change: Change to right, stay in line till change point",
            )

    def terminate(self, new_status):
        pass


class Wait(py_trees.behaviour.Behaviour):
    """
    This behavior handles the waiting in front of the lane change.
    """

    def __init__(
        self,
        name: str,
        curr_behavior_pub: Publisher,
        stop_client: Client,
    ):
        super().__init__(name)
        self.curr_behavior_pub = curr_behavior_pub
        self.stop_client = stop_client

    def setup(self, **kwargs):
        self.blackboard = py_trees.blackboard.Blackboard()

    def initialise(self):
        get_logger().info("Lane Change Wait")
        self.change_option: Optional[int] = None
        self.change_direction: Optional[LaneFreeDirection] = None
        self.counter_lanefree = 0

    def update(self):
        """
        Waits in front of the lane change until function lane free check
        returns True.

        :return: py_trees.common.Status.RUNNING, while is lane free returns False
                 py_trees.common.Status.SUCCESS, when lane free returns True
        """
        if LANECHANGE_FREE:
            self.curr_behavior_pub.publish(bs.lc_app_free.name)
            return debug_status(
                self.name,
                py_trees.common.Status.SUCCESS,
                "Lanechange free, skipping Wait and exit lane change behavior",
            )

        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(self.name, Status.FAILURE, "Lane Change: Map is None")
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        # Update stopline info
        waypoint: Optional[Waypoint] = self.blackboard.get("/paf/hero/current_waypoint")
        if waypoint is not None:
            self.change_option = waypoint.road_option

            # Check if change is to the left or right lane
            if self.change_option == CarlaRoute.CHANGELANELEFT:
                self.change_direction = LaneFreeDirection.LEFT
            elif self.change_option == CarlaRoute.CHANGELANERIGHT:
                self.change_direction = LaneFreeDirection.RIGHT

        if self.change_option is None or self.change_direction is None:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Lane Change: At least one change parameter is None",
            )

        lc_free, lc_mask = tree.is_lane_free(
            right_lane=self.change_direction.value,
            lane_length=22.5,
            lane_transform=-5.0,
            check_method="fallback",
        )
        if isinstance(lc_mask, shapely.Polygon):
            add_debug_marker(debug_marker(lc_mask, color=LANECHANGE_MARKER_COLOR))
        add_debug_entry(self.name, f"Lane change: Is lane free? {lc_free.name}")
        if lc_free is LaneFreeState.FREE:
            self.counter_lanefree += 1
            # using a counter to account for inconsistencies
            if self.counter_lanefree > 1:
                update_stop_marks(
                    self.stop_client,
                    id=LANECHANGE_STOPMARK_ID,
                    reason="lane not blocked",
                    is_global=False,
                    marks=[],
                )
                self.curr_behavior_pub.publish(bs.lc_wait_free.name)
                return debug_status(
                    self.name,
                    Status.SUCCESS,
                    "Lane Change Wait: Change now clear, changing",
                )
            else:
                self.curr_behavior_pub.publish(bs.lc_wait.name)
                return debug_status(
                    self.name,
                    Status.RUNNING,
                    f"Lane Change Wait: Free with count {self.counter_lanefree}/2",
                )
        else:
            self.curr_behavior_pub.publish(bs.lc_wait.name)
            if lc_free is LaneFreeState.BLOCKED:
                self.counter_lanefree = 0
                return debug_status(
                    self.name,
                    Status.RUNNING,
                    "Lane Change Wait: Lane blocked, reset count, stay in current lane",
                )
            else:
                return debug_status(
                    self.name,
                    Status.RUNNING,
                    "Lane Change Wait: Lane free state unknown, "
                    f"keep count {self.counter_lanefree}/2, stay in current lane",
                )

    def terminate(self, new_status):
        pass


class Change(py_trees.behaviour.Behaviour):
    """
    This behavior handles the switching to a new lane in the
    lane change procedure.
    """

    def __init__(
        self,
        name: str,
        curr_behavior_pub: Publisher,
        stop_client: Client,
    ):
        super().__init__(name)
        self.curr_behavior_pub = curr_behavior_pub
        self.stop_client = stop_client

    def setup(self, **kwargs):
        self.blackboard = py_trees.blackboard.Blackboard()

    def initialise(self):
        get_logger().info("Lane Change: Change to next Lane")
        self.waypoint: Optional[Waypoint] = self.blackboard.get(
            "/paf/hero/current_waypoint"
        )
        self.change_position: Optional[Point] = None
        self.curr_behavior_pub.publish(bs.lc_enter_init.name)

    def update(self):
        """
        Continues driving through the lane change until the vehicle gets
        close enough to the next global way point.
        :return: py_trees.common.Status.RUNNING, if still driving to the next lane
                 py_trees.common.Status.SUCCESS, if lane change finished
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        trajectory_local = self.blackboard.get("/paf/hero/trajectory_local")

        if self.waypoint is None or trajectory_local is None:
            return debug_status(
                self.name, Status.FAILURE, "waypoint or trajectory_local is None"
            )
        else:
            self.change_position = self.waypoint.position

        if self.change_position is None:
            return debug_status(
                self.name, Status.FAILURE, "At least one change parameter is None"
            )

        # get change distance from five meter behind global change point
        # (transfered to local hero coords)
        hero_transform = get_global_hero_transform()
        local_pos: Point2 = (
            hero_transform.inverse()
            * Point2.new(self.change_position.x, self.change_position.y)
            + Vector2.forward() * 5.0
        )
        trajectory_local = mapping_common.mask.ros_path_to_line(trajectory_local)
        change_distance = trajectory_local.line_locate_point(local_pos.to_shapely())

        if change_distance > 0:
            return debug_status(
                self.name,
                Status.RUNNING,
                "Lane Change Change: Driving to the next lane and end of lanechange",
            )
        else:
            self.curr_behavior_pub.publish(bs.lc_exit.name)
            # delete stop marks just in case
            update_stop_marks(
                self.stop_client,
                id=LANECHANGE_STOPMARK_ID,
                reason="lane not blocked",
                is_global=False,
                marks=[],
            )
            return debug_status(
                self.name, Status.FAILURE, "Lane Change Change: Finished"
            )

    def terminate(self, new_status):
        pass

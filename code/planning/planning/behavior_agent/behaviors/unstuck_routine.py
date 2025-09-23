import py_trees
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
from typing import Optional

from rclpy.client import Client
from rclpy.publisher import Publisher
from rclpy.clock import Clock
from rclpy.duration import Duration

import mapping_common.mask
from mapping_common.map import Map, MapTree
from mapping_common.entity import Entity
from mapping_common.entity import FlagFilter
from planning.local_planner.utils import get_distance
from planning.behavior_agent.blackboard_utils import Blackboard

from . import behavior_names as bs
from .topics2blackboard import BLACKBOARD_MAP_ID
from .speed_alteration import add_speed_override
from .debug_markers import add_debug_marker, debug_status, debug_marker
from .overtake_service_utils import (
    request_start_overtake,
    request_end_overtake,
)
from .stop_mark_service_utils import (
    update_stop_marks,
)
from . import get_logger

TRIGGER_STUCK_SPEED = 0.1  # default 0.1 (m/s)
TRIGGER_STUCK_DURATION = Duration(seconds=8.0)  # default 8 (s)
TRIGGER_WAIT_STUCK_DURATION = Duration(seconds=15.0)  # default 25 (s)
UNSTUCK_DRIVE_DURATION = Duration(seconds=5.0)  # default 1.2 (s)
UNSTUCK_CLEAR_DISTANCE = 2.5  # default 1.5 (m)
REVERSE_COLLISION_MARKER_COLOR = (209 / 255, 134 / 255, 0 / 255, 1.0)
REVERSE_LOOKUP_DISTANCE = 1.0  # Distance that should be checked behind the car (m)
REVERSE_LOOKUP_WIDTH_FACTOR = 1.25


def duration_secs(d: Duration) -> float:
    """Returns the duration as float in seconds

    Args:
        d (Duration): duration

    Returns:
        float: duration in seconds
    """
    nanosecs = d.nanoseconds
    return nanosecs * 1e-9


def pos_to_array(pos: PoseStamped):
    return np.array([pos.pose.position.x, pos.pose.position.y])


def calculate_obstacle_behind(
    tree: MapTree,
    hero: Entity,
    overlap_percent: float,
) -> bool:
    """Calculates if there is an obstacle behind the vehicle

    Returns:
        bool
    """
    # data preparation
    hero_width = hero.get_width()

    collision_mask = mapping_common.mask.project_plane(
        -(hero.get_front_x() + REVERSE_LOOKUP_DISTANCE),
        hero_width * REVERSE_LOOKUP_WIDTH_FACTOR,
    )
    add_debug_marker(debug_marker(collision_mask, color=REVERSE_COLLISION_MARKER_COLOR))

    entity_result = tree.get_nearest_entity(
        collision_mask, hero.to_shapely(), min_coverage_percent=overlap_percent
    )

    if entity_result is not None:
        entity, _ = entity_result
        add_debug_marker(
            debug_marker(
                entity.entity, color=REVERSE_COLLISION_MARKER_COLOR, scale_z=0.3
            )
        )
        return True
    else:
        return False


class UnstuckRoutine(py_trees.behaviour.Behaviour):
    """
    This behavior is triggered when the vehicle is stuck and needs to be
    unstuck.

    Documentation to this behavior can be found in
    /doc/planning/behaviors/Unstuck.md
    """

    def __init__(
        self,
        name: str,
        clock: Clock,
        curr_behavior_pub: Publisher,
        start_overtake_client: Client,
        end_overtake_client: Client,
        stop_client: Client,
    ):
        super().__init__(name)
        self.clock = clock
        self.curr_behavior_pub = curr_behavior_pub
        self.start_overtake_client = start_overtake_client
        self.end_overtake_client = end_overtake_client
        self.stop_client = stop_client

    def setup(self, **kwargs):
        self.blackboard = Blackboard()
        self.stuck_timer = self.clock.now()
        self.wait_stuck_timer = self.clock.now()
        self.unstuck_count = 0
        self.init_pos = np.array([np.inf, np.inf])

    def initialise(self):
        global TRIGGER_WAIT_STUCK_DURATION
        self.STUCK_DETECTED = False
        current_pos = self.blackboard.try_get("/paf/hero/current_pos")
        current_speed = self.blackboard.try_get("/carla/hero/Speed")
        target_speed = self.blackboard.try_get("/paf/hero/target_velocity")
        curr_behavior = self.blackboard.try_get("/paf/hero/curr_behavior")

        # check for None values and return if so
        if current_speed is None or target_speed is None or current_pos is None:
            get_logger().info(
                f"Available: current_speed: {current_speed is not None}, "
                f"target_speed: {target_speed is not None}, "
                f"current_pos: {current_pos is not None}",
                throttle_duration_sec=2,
            )
            return

        # check if vehicle is NOT stuck, v >= TRIGGER_STUCK_SPEED
        if current_speed.speed >= TRIGGER_STUCK_SPEED:
            # reset wait stuck timer
            self.wait_stuck_timer = self.clock.now()

        # check if vehicle is NOT stuck, v >= TRIGGER_STUCK_SPEED when should
        # have v_target > TRIGGER_STUCK_SPEED or if we should stand and stand
        if (
            current_speed.speed >= TRIGGER_STUCK_SPEED
            and target_speed.data >= TRIGGER_STUCK_SPEED
        ) or (
            current_speed.speed < TRIGGER_STUCK_SPEED
            and target_speed.data < TRIGGER_STUCK_SPEED
        ):
            # reset stuck timer
            self.stuck_timer = self.clock.now()

        # If we drove for more than 15 meter since last unstuck attempt
        # --> indicates new stuck location --> reset unstuck_count
        current_pos = pos_to_array(current_pos)
        if get_distance(self.init_pos, current_pos) > 15:
            self.unstuck_count = 0

        # when no curr_behavior (before unparking lane free) or
        # a wait behavior occurs, increase the wait stuck duration
        wait_behaviors = [bs.lc_wait.name, bs.ot_wait.name]
        wait_long_behaviors = [bs.int_wait.name, bs.int_app_to_stop.name]

        last_duration = TRIGGER_WAIT_STUCK_DURATION
        if self.unstuck_count != 0:
            TRIGGER_WAIT_STUCK_DURATION = Duration(seconds=5.0)
        elif curr_behavior is None or curr_behavior.data in wait_behaviors:
            TRIGGER_WAIT_STUCK_DURATION = Duration(seconds=30.0)
        elif curr_behavior.data in wait_long_behaviors:
            TRIGGER_WAIT_STUCK_DURATION = Duration(seconds=60.0)
        else:
            TRIGGER_WAIT_STUCK_DURATION = Duration(seconds=15.0)

        # Set back timer if the duration just got smaller
        if TRIGGER_WAIT_STUCK_DURATION < last_duration:
            # reset both timers
            self.stuck_timer = self.clock.now()
            self.wait_stuck_timer = self.clock.now()

        # update the stuck durations
        self.stuck_duration = self.clock.now() - self.stuck_timer
        self.wait_stuck_duration = self.clock.now() - self.wait_stuck_timer

        if (
            self.stuck_duration >= TRIGGER_STUCK_DURATION
            or self.wait_stuck_duration >= TRIGGER_WAIT_STUCK_DURATION
        ):
            self.STUCK_DETECTED = True
            self.init_pos = current_pos
            self.init_ros_stuck_time = self.clock.now()
            stuck_reason = "Stuck"
            stuck_dur = duration_secs(TRIGGER_STUCK_DURATION)
            if self.wait_stuck_duration >= TRIGGER_WAIT_STUCK_DURATION:
                stuck_reason = "Wait Stuck"
                stuck_dur = duration_secs(TRIGGER_WAIT_STUCK_DURATION)
            get_logger().fatal(
                f"{stuck_reason} in one place for more than "
                f"{stuck_dur:.2f} sec --> starting unstuck routine"
            )

    def update(self):
        """
        This behaviour doesn't do anything else than just keep running unless
        there is a higher priority behaviour

        :return: py_trees.common.Status.RUNNING, unstuck routine running for
        UNSTUCK_DRIVE_DURATION
        :return: py_trees.common.Status.FAILURE, unstuck routine finished or
        not need to be triggered
        """
        current_pos = self.blackboard.try_get("/paf/hero/current_pos")
        current_speed = self.blackboard.try_get("/carla/hero/Speed")
        map: Optional[Map] = self.blackboard.try_get(BLACKBOARD_MAP_ID)

        if current_pos is None or current_speed is None or map is None:
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                "map, current_pos or current_speed is None",
            )

        current_pos = pos_to_array(current_pos)

        # if no stuck detected, return failure
        if not self.STUCK_DETECTED:
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                f"No stuck detected.\n"
                f"stuck_dur: {duration_secs(self.stuck_duration):.2f}/"
                f"{duration_secs(TRIGGER_STUCK_DURATION):.2f}, "
                f"wait_stuck_dur: {duration_secs(self.wait_stuck_duration):.2f}/"
                f"{duration_secs(TRIGGER_WAIT_STUCK_DURATION):.2f}",
            )

        curr_us_drive_dur = self.clock.now() - self.init_ros_stuck_time
        # stuck detected, starting unstuck routine for UNSTUCK_DRIVE_DURATION seconds
        if curr_us_drive_dur < UNSTUCK_DRIVE_DURATION:
            self.curr_behavior_pub.publish(String(data=bs.us_unstuck.name))
            tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))
            hero: Optional[Entity] = tree.map.hero()
            if hero is None:
                return debug_status(
                    self.name, py_trees.common.Status.FAILURE, "hero is None"
                )
            collision_detected = calculate_obstacle_behind(tree, hero, 0.1)
            if (
                get_distance(self.init_pos, current_pos) < UNSTUCK_CLEAR_DISTANCE
            ) and not collision_detected:
                add_speed_override(-2.0)
            elif get_distance(self.init_pos, current_pos) < 0.5:
                add_speed_override(-2.0)
            else:
                # skip waiting till UNSTUCK_DRIVE_DURATION reached
                self.init_ros_stuck_time -= UNSTUCK_DRIVE_DURATION - curr_us_drive_dur
                add_speed_override(0.0)
            return debug_status(
                self.name,
                py_trees.common.Status.RUNNING,
                "Unstuck routine running. Try driving backward.",
            )
        # drive for UNSTUCK_DRIVE_DURATION forwards again
        # (to pass stopmarkers before they are set again)
        elif curr_us_drive_dur < 2.0 * UNSTUCK_DRIVE_DURATION:
            self.curr_behavior_pub.publish(String(data=bs.us_forward.name))
            if self.unstuck_count == 1:
                request_end_overtake(self.end_overtake_client)
                update_stop_marks(
                    self.stop_client,
                    id="unstuck",
                    reason="unstuck triggered",
                    is_global=False,
                    marks=[],
                    delete_all_others=True,
                )
            elif self.unstuck_count == 2:
                request_start_overtake(
                    self.start_overtake_client, start_transition_length=0.0
                )
            elif self.unstuck_count == 3:
                request_start_overtake(
                    self.start_overtake_client, start_transition_length=0.0, offset=-1.0
                )
                self.unstuck_count = 0
            return debug_status(
                self.name,
                py_trees.common.Status.RUNNING,
                "Unstuck routine running. Try driving forward.",
            )
        else:
            add_speed_override(0.0)
            request_end_overtake(self.end_overtake_client)
            self.curr_behavior_pub.publish(String(data=bs.us_stop.name))
            self.stuck_timer = self.clock.now()
            self.wait_stuck_timer = self.clock.now()
            self.STUCK_DETECTED = False
            self.unstuck_count += 1
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                f"Unstuck routine ran for "
                f"{2.0 * duration_secs(UNSTUCK_DRIVE_DURATION):.2f}. Exiting.",
            )

    def terminate(self, new_status):
        pass

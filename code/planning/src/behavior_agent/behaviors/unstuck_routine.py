import py_trees
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
import numpy as np
from typing import Optional
import mapping_common.mask
from mapping_common.map import Map, MapTree
from .topics2blackboard import BLACKBOARD_MAP_ID
from mapping_common.entity import Entity
from mapping_common.entity import FlagFilter
from local_planner.utils import get_distance
from . import behavior_names as bs
from .speed_alteration import add_speed_override
from .debug_markers import add_debug_marker, debug_status, debug_marker
from .overtake_service_utils import (
    request_start_overtake,
    create_start_overtake_proxy,
    request_end_overtake,
    create_end_overtake_proxy,
)
from .stop_mark_service_utils import (
    create_stop_marks_proxy,
    update_stop_marks,
)

TRIGGER_STUCK_SPEED = 0.1  # default 0.1 (m/s)
TRIGGER_STUCK_DURATION = rospy.Duration(8)  # default 8 (s)
TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(15)  # default 25 (s)
UNSTUCK_DRIVE_DURATION = rospy.Duration(5)  # default 1.2 (s)
UNSTUCK_CLEAR_DISTANCE = 2.5  # default 1.5 (m)
REVERSE_COLLISION_MARKER_COLOR = (209 / 255, 134 / 255, 0 / 255, 1.0)
REVERSE_LOOKUP_DISTANCE = 1.0  # Distance that should be checked behind the car (m)
REVERSE_LOOKUP_WIDTH_FACTOR = 1.25


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

    def __init__(self, name):
        super(UnstuckRoutine, self).__init__(name)
        rospy.loginfo("Unstuck Init")

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/curr_behavior", String, queue_size=1
        )
        self.start_overtake_proxy = create_start_overtake_proxy()
        self.end_overtake_proxy = create_end_overtake_proxy()
        self.stop_proxy = create_stop_marks_proxy()
        self.stuck_timer = rospy.Time.now()
        self.wait_stuck_timer = rospy.Time.now()
        self.unstuck_count = 0
        self.init_pos = np.array([np.inf, np.inf])
        return True

    def initialise(self):
        global TRIGGER_WAIT_STUCK_DURATION
        self.STUCK_DETECTED = False
        current_pos = self.blackboard.get("/paf/hero/current_pos")
        current_speed = self.blackboard.get("/carla/hero/Speed")
        target_speed = self.blackboard.get("/paf/hero/target_velocity")
        curr_behavior = self.blackboard.get("/paf/hero/curr_behavior")

        # check for None values and return if so
        if current_speed is None or target_speed is None or current_pos is None:
            rospy.logdebug("current_speed, target_speed or current_pos is None")
            return

        # check if vehicle is NOT stuck, v >= TRIGGER_STUCK_SPEED
        if current_speed.speed >= TRIGGER_STUCK_SPEED:
            # reset wait stuck timer
            self.wait_stuck_timer = rospy.Time.now()

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
            self.stuck_timer = rospy.Time.now()

        # when no curr_behavior (before unparking lane free) or
        # a wait behavior occurs, increase the wait stuck duration
        wait_behaviors = [bs.lc_wait.name, bs.ot_wait.name]
        wait_long_behaviors = [bs.int_wait.name, bs.int_app_to_stop.name]

        if curr_behavior is None or curr_behavior.data in wait_behaviors:
            TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(30)
        elif curr_behavior.data in wait_long_behaviors:
            TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(60)
        else:
            TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(15)

        # update the stuck durations
        self.stuck_duration = rospy.Time.now() - self.stuck_timer
        self.wait_stuck_duration = rospy.Time.now() - self.wait_stuck_timer

        if (
            self.stuck_duration >= TRIGGER_STUCK_DURATION
            or self.wait_stuck_duration >= TRIGGER_WAIT_STUCK_DURATION
        ):
            self.STUCK_DETECTED = True
            self.unstuck_count += 1
            request_end_overtake(self.end_overtake_proxy)
            update_stop_marks(
                self.stop_proxy,
                id="unstuck",
                reason="unstuck triggered",
                is_global=False,
                marks=[],
                delete_all_others=True,
            )
            # If we drove for more than 20 meter since last unstuck attempt
            # --> indicates new stuck location --> reset unstuck_count
            current_pos = pos_to_array(current_pos)
            if get_distance(self.init_pos, current_pos) > 20:
                self.unstuck_count = 0
            self.init_pos = current_pos
            self.init_ros_stuck_time = rospy.Time.now()
            stuck_reason = "Stuck"
            stuck_dur = TRIGGER_STUCK_DURATION.secs
            if self.wait_stuck_duration >= TRIGGER_WAIT_STUCK_DURATION:
                stuck_reason = "Wait Stuck"
                stuck_dur = TRIGGER_WAIT_STUCK_DURATION.secs
            rospy.logfatal(
                f"{stuck_reason} in one place for more than "
                f"{stuck_dur} sec --> starting unstuck routine"
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
        current_pos = self.blackboard.get("/paf/hero/current_pos")
        current_speed = self.blackboard.get("/carla/hero/Speed")
        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)

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
                f"stuck_dur: {self.stuck_duration.secs}/{TRIGGER_STUCK_DURATION.secs}, "
                f"wait_stuck_dur: {self.wait_stuck_duration.secs}/"
                f"{TRIGGER_WAIT_STUCK_DURATION.secs}",
            )

        curr_us_drive_dur = rospy.Time.now() - self.init_ros_stuck_time
        # stuck detected, starting unstuck routine for UNSTUCK_DRIVE_DURATION seconds
        if curr_us_drive_dur < UNSTUCK_DRIVE_DURATION:
            self.curr_behavior_pub.publish(bs.us_unstuck.name)
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
        elif curr_us_drive_dur < 2 * UNSTUCK_DRIVE_DURATION:
            self.curr_behavior_pub.publish(bs.us_forward.name)
            if self.unstuck_count == 3:
                request_start_overtake(
                    self.start_overtake_proxy, start_transition_length=0.0
                )
            elif self.unstuck_count == 4:
                request_start_overtake(
                    self.start_overtake_proxy, start_transition_length=0.0, offset=-1.0
                )
                self.unstuck_count = 0
            return debug_status(
                self.name,
                py_trees.common.Status.RUNNING,
                "Unstuck routine running. Try driving forward.",
            )
        else:
            add_speed_override(0.0)
            request_end_overtake(self.end_overtake_proxy)
            self.curr_behavior_pub.publish(bs.us_stop.name)
            self.stuck_timer = rospy.Time.now()
            self.wait_stuck_timer = rospy.Time.now()
            self.STUCK_DETECTED = False
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                f"Unstuck routine ran for {2 * UNSTUCK_DRIVE_DURATION.secs}. Exiting.",
            )

    def terminate(self, new_status):
        pass

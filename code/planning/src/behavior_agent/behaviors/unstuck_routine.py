import py_trees
import rospy
from std_msgs.msg import String, Float32, Bool
import numpy as np
from typing import Optional
import mapping_common.mask
from mapping_common.map import Map, MapTree
from .topics2blackboard import BLACKBOARD_MAP_ID
from mapping_common.entity import Entity
from mapping_common.entity import FlagFilter
from . import behavior_speed as bs
from .speed_alteration import add_speed_override
from .debug_markers import add_debug_marker, debug_status, debug_marker
from .overtake_service_utils import request_end_overtake, create_end_overtake_proxy
from .stop_mark_service_utils import (
    create_stop_marks_proxy,
    update_stop_marks,
)

TRIGGER_STUCK_SPEED = 0.1  # default 0.1 (m/s)
TRIGGER_STUCK_DURATION = rospy.Duration(8)  # default 8 (s)
TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(25)  # default 25 (s)
UNSTUCK_DRIVE_DURATION = rospy.Duration(5)  # default 1.2 (s)
UNSTUCK_CLEAR_DISTANCE = 1.5  # default 1.5 (m)
REVERSE_COLLISION_MARKER_COLOR = (209 / 255, 134 / 255, 0 / 255, 1.0)
REVERSE_LOOKUP_DISTANCE = 1.0  # Distance that should be checked behind the car (m)
REVERSE_LOOKUP_WIDTH_FACTOR = 1.25
# STUCK_DETECTED = False


def get_distance(pos_1, pos_2):
    """Calculate the distance between two positions

    Args:
        pos1 (np.array): Position 1 [#,#]
        pos2 (np.array): Position 2 [#,#]

    Returns:
        float: Distance
    """

    return np.linalg.norm(pos_1 - pos_2)


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
    /doc/planning/Behavior_detailed.md
    """

    # def print_warnings(self):
    # update last log values
    #    if self.stuck_duration.secs > TRIGGER_STUCK_DURATION.secs / 2:
    #        rospy.logwarn_throttle(1, f"Stuck for {self.stuck_duration.secs} s")
    #    if self.wait_stuck_duration.secs > TRIGGER_WAIT_STUCK_DURATION.secs / 2:
    #        rospy.logwarn_throttle(
    #            1, f"Wait stuck for {self.wait_stuck_duration.secs} s"
    #        )

    def __init__(self, name):
        super(UnstuckRoutine, self).__init__(name)
        self.stuck_timer = rospy.Time.now()
        self.wait_stuck_timer = rospy.Time.now()
        rospy.loginfo("Unstuck Init")

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/curr_behavior", String, queue_size=1
        )
        # self.pub_unstuck_distance = rospy.Publisher(
        #    "/paf/hero/unstuck_distance", Float32, queue_size=1
        # )
        # self.pub_unstuck_flag = rospy.Publisher(
        #    "/paf/hero/unstuck_flag", Bool, queue_size=1
        # )
        return True

    def initialise(self):
        global TRIGGER_WAIT_STUCK_DURATION
        # global STUCK_DETECTED
        self.STUCK_DETECTED = False

        # self.unstuck_overtake_count = 0
        # self.last_unstuck_positions: Optional[np.ndarray] = None
        # above was before: np.array([np.array([0, 0]), np.array([0, 0])])
        current_pos = self.blackboard.get("/paf/hero/current_pos")
        current_speed = self.blackboard.get("/carla/hero/Speed")
        target_speed = self.blackboard.get("/paf/hero/target_velocity")
        curr_behavior = self.blackboard.get("/paf/hero/curr_behavior")

        # check for None values and initialize if necessary
        if current_speed is None or target_speed is None or current_pos is None:
            rospy.logdebug("current_speed, target_speed or current_pos is None")
            return

        self.init_pos = np.array(
            [current_pos.pose.position.x, current_pos.pose.position.y]
        )
        self.init_ros_stuck_time = rospy.Time.now()

        # check if vehicle is NOT stuck, v > 0.1
        if current_speed.speed >= TRIGGER_STUCK_SPEED:
            # reset wait stuck timer
            self.wait_stuck_timer = rospy.Time.now()

            # check if vehicle is NOT stuck, v >= 0.1 when should be v > 0.1
            if target_speed.data >= TRIGGER_STUCK_SPEED:
                # reset stuck timer
                self.stuck_timer = rospy.Time.now()

        # when no curr_behavior (before unparking lane free) or
        # a wait behavior occurs, increase the wait stuck duration
        wait_behaviors = [bs.lc_wait.name, bs.ot_app_blocked.name]
        wait_long_behaviors = [bs.int_wait.name]

        if curr_behavior is None or curr_behavior.data in wait_behaviors:
            TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(50)
        elif curr_behavior.data in wait_long_behaviors:
            TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(75)
        else:
            TRIGGER_WAIT_STUCK_DURATION = rospy.Duration(25)

        # update the stuck durations
        self.stuck_duration = rospy.Time.now() - self.stuck_timer
        self.wait_stuck_duration = rospy.Time.now() - self.wait_stuck_timer

        # print warnings to indicate potential stuck
        # self.print_warnings()

        if (
            self.stuck_duration >= TRIGGER_STUCK_DURATION
            or self.wait_stuck_duration >= TRIGGER_WAIT_STUCK_DURATION
        ):
            self.STUCK_DETECTED = True
            end_overtake_proxy = create_end_overtake_proxy()
            request_end_overtake(end_overtake_proxy)
            stop_proxy = create_stop_marks_proxy()
            update_stop_marks(
                stop_proxy,
                id="unstuck",
                reason="unstuck triggered",
                is_global=False,
                marks=[],
                delete_all_others=True,
            )
            stuck_reason = "Stuck"
            if self.wait_stuck_duration >= TRIGGER_WAIT_STUCK_DURATION:
                stuck_reason = "Wait Stuck"
            rospy.logfatal(
                f"{stuck_reason} in one place for more than "
                f"{TRIGGER_STUCK_DURATION.secs} sec\n"
                " --> starting unstuck routine"
            )
        return True

    def update(self):
        """
        This behaviour doesn't do anything else than just keep running unless
        there is a higher priority behaviour

        :return: py_trees.common.Status.RUNNING, keeps the decision tree from
        finishing
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

        # if no stuck detected, return failure
        if not self.STUCK_DETECTED:
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                f"No stuck detected.\nstuck_dur: {self.stuck_duration.secs}, "
                f"wait_stuck_dur: {self.wait_stuck_duration.secs}",
            )
            # self.pub_unstuck_flag.publish(False)
            # unstuck distance -1 is set, to reset the unstuck distance
            # self.pub_unstuck_distance.publish(-1)

        # stuck detected, starting unstuck routine for UNSTUCK_DRIVE_DURATION seconds
        if rospy.Time.now() - self.init_ros_stuck_time < UNSTUCK_DRIVE_DURATION:
            self.curr_behavior_pub.publish(bs.us_unstuck.name)
            # self.pub_unstuck_flag.publish(True)
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
                add_speed_override(-0.05)
            elif get_distance(self.init_pos, current_pos) < 0.5:
                add_speed_override(-0.05)
            else:
                add_speed_override(0.001)
            return debug_status(
                self.name,
                py_trees.common.Status.RUNNING,
                "Unstuck routine running.",
            )
        else:
            self.STUCK_DETECTED = False
            add_speed_override(0.001)
            self.curr_behavior_pub.publish(bs.us_stop.name)
            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                f"Unstuck routine ran for {UNSTUCK_DRIVE_DURATION.secs}. Exiting.",
            )

            # while vehicle is stopping publish us_stop
            # if abs(current_speed.speed) > 0.1:

            #    return py_trees.common.Status.RUNNING
            # vehicle has stopped:
            # unstuck_distance = get_distance(self.init_pos, current_pos)
            # self.pub_unstuck_distance.publish(unstuck_distance)

            # check if vehicle needs to overtake:
            # save current pos to last_unstuck_positions
            # self.last_unstuck_positions = np.roll(
            #    self.last_unstuck_positions, -1, axis=0
            # )
            # self.last_unstuck_positions[-1] = self.init_pos

            # if last unstuck was too far away, no overtake
            # we only want to overtake when we tried to unstuck twice
            # this case is the first time ever we tried to unstuck
            # if np.array_equal(self.last_unstuck_positions[0], np.array([0, 0])):
            #    self.reset_stuck_values()
            #    rospy.logwarn("Unstuck routine finished.")
            #    return py_trees.common.Status.FAILURE
            # rospy.logfatal("Distance to last unstuck position: %s",
            #                get_distance(self.last_unstuck_positions[0],
            #                             self.last_unstuck_positions[-1]))
            # if the distance between the last and the first unstuck position
            # is too far, we don't want to overtake, since its the first
            # unstuck routine at this position on the map
            # if (
            #    get_distance(
            #        self.last_unstuck_positions[0], self.last_unstuck_positions[-1]
            #    )
            #    > UNSTUCK_CLEAR_DISTANCE
            # ):
            #    self.reset_stuck_values()
            #    rospy.logwarn("Unstuck routine finished.")
            #    return py_trees.common.Status.FAILURE

            # once we tried the unstuck twice, we try to overtake
            # if current_speed.speed < 1:
            # rospy.logwarn("Unstuck DISTANCE %s.", unstuck_distance)

            # publish the over take behavior 3 times to make sure
            # it is detected
            #    self.curr_behavior_pub.publish(bs.us_overtake.name)
            #    if self.unstuck_overtake_count > 3:
            #        self.reset_stuck_values()
            #        rospy.logwarn("Unstuck routine finished.")
            #        return py_trees.common.Status.FAILURE
            #    else:
            #        self.unstuck_overtake_count += 1
            #        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        pass

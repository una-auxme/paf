import py_trees
from py_trees.common import Status
from typing import Optional
from std_msgs.msg import String, Float32

import rospy
import numpy as np

# import carla
from agents.navigation.local_planner import RoadOption

from mapping_common.map import Map, LaneFreeState
from mapping_common.entity import FlagFilter
from mapping_common.markers import debug_marker
import shapely
from . import behavior_speed as bs
from .topics2blackboard import BLACKBOARD_MAP_ID
from .debug_markers import add_debug_marker, debug_status, add_debug_entry

from local_planner.utils import TARGET_DISTANCE_TO_STOP_LANECHANGE, convert_to_ms

LANECHANGE_MARKER_COLOR = (153 / 255, 50 / 255, 204 / 255, 1.0)
LANECHANGE_FREE = False

"""
Source: https://github.com/ll7/psaf2
"""


class Ahead(py_trees.behaviour.Behaviour):
    """
    This behaviour checks whether there is a lane change in front of the
    ego vehicle or not and triggers the rest of the decision tree handling the
    lane change.
    """

    def __init__(self, name):
        super(Ahead, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        self.dist = 0

    def update(self):
        """
        Gets the current distance to the next lane change.
        :return: py_trees.common.Status.SUCCESS, if the vehicle is within range
                 of the lane change
                 py_trees.common.Status.FAILURE, if we are too far away from
                 the lane change
        """

        lane_change_distance = self.blackboard.get("/paf/hero/lane_change_distance")
        if lane_change_distance is None:
            return debug_status(
                self.name, Status.FAILURE, "lane_change_distance is None"
            )
        else:
            change_distance = lane_change_distance.distance
            change_detected = lane_change_distance.isLaneChange
            # add_debug_entry(
            #    self.name,
            #    f"road option: {lane_change_distance.roadOption} "
            #    f"RoadOption.CHANGELANELEFT: {RoadOption.CHANGELANELEFT} "
            #    f"RoadOption.CHANGELANERIGHT: {RoadOption.CHANGELANERIGHT} ",
            # )
        if change_detected and change_distance < 30:
            return debug_status(self.name, Status.SUCCESS, "Lane change ahead")
        else:
            return debug_status(self.name, Status.FAILURE, "No lane change ahead")

    def terminate(self, new_status):
        pass


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an lane change and behaviours.road_features.LaneChangeAhead is
    triggered. It than handles approaching the lane change, slowing the
    vehicle down appropriately.
    """

    def __init__(self, name):
        super(Approach, self).__init__(name)
        rospy.loginfo("Init -> Lane Change Behavior: Approach")

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.ot_distance_pub = rospy.Publisher(
            "/paf/hero/" "overtake_distance", Float32, queue_size=1
        )
        return True

    def initialise(self):
        rospy.loginfo("Approaching Change")
        global LANECHANGE_FREE
        LANECHANGE_FREE = False
        self.change_detected = False
        self.change_distance: Optional[float] = None
        self.change_option: Optional[RoadOption] = None
        self.change_direction: Optional[bool] = None
        # self.virtual_change_distance: Optional[float] = None
        self.blocked = True
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

        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(self.name, Status.FAILURE, "Map is None")
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        # Get lane change distance waypoint from blackboard
        lane_change_distance = self.blackboard.get("/paf/hero/lane_change_distance")
        if lane_change_distance is not None:
            self.change_distance = lane_change_distance.distance
            self.change_detected = lane_change_distance.isLaneChange
            self.change_option = lane_change_distance.roadOption

            # Check if change is to the left or right lane
            if self.change_option == RoadOption.CHANGELANELEFT:
                self.change_direction = False
            elif self.change_option == RoadOption.CHANGELANERIGHT:
                self.change_direction = True

        if (
            self.change_distance is None
            or self.change_option is None
            or self.change_direction is None
        ):
            return debug_status(
                self.name, Status.FAILURE, "At least one change parameter is None"
            )

        add_debug_entry(
            self.name,
            f"Change distance: {self.change_distance}\n",
        )
        add_debug_entry(
            self.name,
            f"Change direction: {'right' if self.change_direction else 'left'}\n",
        )

        # calculate virtual stopline
        if self.change_detected:

            lc_free, lc_mask = tree.is_lane_free(
                right_lane=self.change_direction,
                lane_length=22.5,
                lane_transform=-5.0,
                check_method="fallback",
            )
            # lc_free = LaneFreeState.FREE
            if isinstance(lc_mask, shapely.Polygon):
                add_debug_marker(debug_marker(lc_mask, color=LANECHANGE_MARKER_COLOR))
            add_debug_entry(self.name, f"Lanechange free?: {lc_free.name}")
            if lc_free is LaneFreeState.FREE:
                self.counter_lanefree += 1
                if self.counter_lanefree > 0:
                    add_debug_entry(self.name, "Lane Change is free not slowing down!")
                    self.ot_distance_pub.publish(self.change_distance * 2.0)
                    self.curr_behavior_pub.publish(bs.lc_app_free.name)
                    self.blocked = False
                    LANECHANGE_FREE = True
                    return debug_status(
                        self.name, Status.SUCCESS, "Lane Change Approach: Lane free"
                    )
                else:
                    add_debug_entry(
                        self.name, f"Lane Change free count: {self.counter_lanefree}"
                    )
                    self.blocked = True
                    # self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
                    # return debug_status(
                    #    self.name,
                    #    Status.RUNNING,
                    #    f"Lane Change free count: {self.counter_lanefree}",
                    # )
            else:
                self.blocked = True
                if lc_free is LaneFreeState.BLOCKED:
                    self.counter_lanefree = 0
                    self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
                add_debug_entry(
                    self.name, "Lane Change Approach: Oncoming blocked slowing down"
                )

        # rospy.loginfo(f"Lane Change approach counter: {self.counter_lanefree}")

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Lane Change Approach: no speedometer connected",
            )

        target_distance = TARGET_DISTANCE_TO_STOP_LANECHANGE
        if self.change_distance <= target_distance:
            return debug_status(
                self.name,
                Status.SUCCESS,
                f"Lane Change Approach: Reached target distance {self.change_distance},"
                " stopping approach",
            )
        else:
            return debug_status(
                self.name, Status.RUNNING, "Lane Change Approach: still approaching"
            )

    """
        if self.change_distance > target_distance and self.blocked:
            # too far
            # self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
            return debug_status(
                self.name,
                Status.RUNNING,
                f"Lane Change Approach: still approaching, "
                f"distance: {self.change_distance}",
            )

        elif (
            speed <= convert_to_ms(2.0)
            and self.change_distance < target_distance
            and self.blocked
        ):
            # stopped
            return debug_status(
                self.name,
                Status.SUCCESS,
                "Lane Change Approach: stopped at virtual stop line",
            )
        elif (
            speed > convert_to_ms(2.0)
            and self.change_distance < target_distance
            and not self.blocked
        ):
            return debug_status(
                self.name,
                Status.SUCCESS,
                "Lane Change Approach: driven over virtual stop line",
            )
        else:
            return debug_status(
                self.name, Status.RUNNING, "Lane Change Approach: still approaching"
            )
        """

    def terminate(self, new_status):
        pass


class Wait(py_trees.behaviour.Behaviour):
    """
    This behavior handles the waiting in front of the lane change.
    """

    def __init__(self, name):
        super(Wait, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        return True

    def initialise(self):
        rospy.loginfo("Lane Change Wait")
        self.change_option: Optional[RoadOption] = None
        self.change_direction: Optional[bool] = None
        self.counter_lanefree = 0
        return True

    def update(self):
        """
        Waits in front of the lane change until function lane free check
        returns True.

        :return: py_trees.common.Status.RUNNING, while is lane free returns False
                 py_trees.common.Status.SUCCESS, when lane free returns True
        """
        global LANECHANGE_FREE

        if LANECHANGE_FREE:
            self.curr_behavior_pub.publish(bs.lc_app_free.name)
            return debug_status(
                self.name, py_trees.common.Status.SUCCESS, "LaneChange free"
            )

        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(self.name, Status.FAILURE, "Map is None")
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        # Update stopline info
        lane_change_distance = self.blackboard.get("/paf/hero/lane_change_distance")
        if lane_change_distance is not None:
            self.change_option = lane_change_distance.roadOption

            # Check if change is to the left or right lane
            if self.change_option == RoadOption.CHANGELANELEFT:
                self.change_direction = False
            elif self.change_option == RoadOption.CHANGELANERIGHT:
                self.change_direction = True

        if self.change_option is None or self.change_direction is None:
            return debug_status(
                self.name, Status.FAILURE, "At least one change parameter is None"
            )

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Lane Change Wait: no speedometer connected",
            )

        if speed > convert_to_ms(10):
            return debug_status(
                self.name,
                Status.SUCCESS,
                "Lane Change Wait: Was not blocked, proceed to drive forward",
            )

        lc_free, lc_mask = tree.is_lane_free(
            right_lane=self.change_direction,
            lane_length=22.5,
            lane_transform=-5.0,
            check_method="lanemarking",
        )
        if isinstance(lc_mask, shapely.Polygon):
            add_debug_marker(debug_marker(lc_mask, color=LANECHANGE_MARKER_COLOR))
        add_debug_entry(self.name, f"Lanechange free?: {lc_free}")
        if lc_free is LaneFreeState.FREE:
            self.counter_lanefree += 1
            if self.counter_lanefree > 0:
                # add_debug_entry(self.name, "Lane Change Wait: Change clear")
                return debug_status(
                    self.name, Status.SUCCESS, "Lane Change Wait: Change clear"
                )
            else:
                self.curr_behavior_pub.publish(bs.lc_wait.name)
                return debug_status(
                    self.name,
                    Status.RUNNING,
                    f"Lane Change Wait: Free count: {self.counter_lanefree}",
                )
        else:
            if lc_free is LaneFreeState.BLOCKED:
                self.counter_lanefree = 0
            self.curr_behavior_pub.publish(bs.lc_wait.name)
            # add_debug_entry(self.name, "Lane Change Wait: Blocked")
            return debug_status(self.name, Status.RUNNING, "Lane Change Wait: Blocked")

    def terminate(self, new_status):
        pass


class Change(py_trees.behaviour.Behaviour):
    """
    This behavior handles the switching to a new lane in the
    lane change procedure.
    """

    def __init__(self, name):
        super(Change, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        return True

    def initialise(self):
        """
        This prints a state status message and changes the driving speed for
        the lane change.
        """
        rospy.loginfo("Lane Change: Change to next Lane")
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
        next_waypoint_msg = self.blackboard.get("/paf/hero/lane_change_distance")

        if next_waypoint_msg is None:
            return debug_status(
                self.name, Status.FAILURE, "Lane Change Change: No new waypoint"
            )
        rospy.logerr(f"lc distance: {next_waypoint_msg.distance}")
        if next_waypoint_msg.distance < 5:
            # rospy.loginfo("Lane Change Change: Drive on the next lane!")
            add_debug_entry(self.name, f"wp distance: {next_waypoint_msg.distance}")
            return debug_status(
                self.name, Status.RUNNING, "Lane Change Change: Drive on the next lane!"
            )
        else:
            self.curr_behavior_pub.publish(bs.lc_exit.name)
            return debug_status(
                self.name, Status.FAILURE, "Lane Change Change: Finished"
            )

    def terminate(self, new_status):
        pass

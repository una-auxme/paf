import py_trees
from py_trees.common import Status
from typing import Optional
from std_msgs.msg import String

import rospy
import numpy as np

import carla
from agents.navigation.local_planner import RoadOption

import mapping_common.map
import mapping_common.entity
from mapping_common.map import Map, LaneFreeState
from mapping_common.entity import FlagFilter
from mapping_common.markers import debug_marker
import shapely
from . import behavior_speed as bs
from .topics2blackboard import BLACKBOARD_MAP_ID
from .debug_markers import add_debug_marker, debug_status, add_debug_entry

from local_planner.utils import TARGET_DISTANCE_TO_STOP, convert_to_ms

LANECHANGE_MARKER_COLOR = (153 / 255, 50 / 255, 204 / 255, 1.0)

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
        return True

    def initialise(self):
        rospy.loginfo("Approaching Change")
        self.change_detected = False
        self.change_distance = np.inf
        self.change_direction: Optional[bool] = None
        self.virtual_change_distance = np.inf
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
        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(self.name, Status.FAILURE, "Map is None")
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        # Update stopline info
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

            add_debug_entry(
                self.name,
                f"Change distance: {self.change_distance}\n"
                f"Change direction right: {self.change_direction}",
            )

        # calculate virtual stopline
        if self.change_detected and self.change_distance != np.inf:
            self.virtual_change_distance = self.change_distance

            if self.change_direction is None:
                return debug_status(
                    self.name, Status.FAILURE, "No lane change direction detected"
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
                if self.counter_lanefree > 3:
                    add_debug_entry(self.name, "Lane Change is free not slowing down!")
                    self.curr_behavior_pub.publish(bs.lc_app_free.name)
                    self.blocked = False
                    return debug_status(self.name, Status.SUCCESS, "Lanechange free")
                else:
                    # self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
                    # self.blocked = True
                    return debug_status(
                        self.name,
                        Status.RUNNING,
                        f"Lane Change free count: {self.counter_lanefree}",
                    )
            else:
                # self.blocked = True
                # self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
                self.counter_lanefree = 0
                add_debug_entry(
                    self.name, "Lane Change Approach: oncoming blocked slowing down"
                )

        # rospy.loginfo(f"Lane Change approach counter: {self.counter_lanefree}")

        # get speed
        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            return debug_status(
                self.name,
                Status.RUNNING,
                "Lane Change Approach: no speedometer connected",
            )

        target_distance = TARGET_DISTANCE_TO_STOP
        if self.virtual_change_distance > target_distance and self.blocked:
            # too far
            self.curr_behavior_pub.publish(bs.lc_app_blocked.name)
            return debug_status(
                self.name,
                Status.RUNNING,
                f"Lane Change Approach: still approaching, "
                f"distance: {self.virtual_change_distance}",
            )

        elif (
            speed < convert_to_ms(2.0)
            and self.virtual_change_distance < target_distance
            and self.blocked
        ):
            # stopped
            return debug_status(
                self.name,
                Status.SUCCESS,
                "Lane Change Approach: stopped at virtual stop line",
            )
        elif (
            speed > convert_to_ms(5.0)
            and self.virtual_change_distance < 3.5
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

    def terminate(self, new_status):
        pass


class Wait(py_trees.behaviour.Behaviour):
    """
    This behavior handles the waiting in front of the lane change.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
         :param name: name of the behaviour
        """
        super(Wait, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the current behavior publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This just prints a state status message.
        """
        rospy.loginfo("Lane Change Wait")
        self.counter_lanefree = 0
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Waits in front of the lane change until the lane change is free.
        :return: py_trees.common.Status.RUNNING, while traffic light is yellow
                 or red
                 py_trees.common.Status.SUCCESS, if the traffic light switched
                 to green or no traffic light is detected
        """

        speedometer = self.blackboard.get("/carla/hero/Speed")
        if speedometer is not None:
            speed = speedometer.speed
        else:
            rospy.logwarn("Lane change wait: no speedometer connected")
            return py_trees.common.Status.RUNNING

        if speed > convert_to_ms(10):
            rospy.loginfo("Lane change wait: Was not blocked, proceed to drive forward")
            return py_trees.common.Status.SUCCESS

        # TODO: ADD FEATURE Check for Traffic
        # distance_lidar = 20

        # change_clear = False
        # if distance_lidar is not None:
        #     # if distance smaller than 15m, change is blocked
        #     if distance_lidar < 15.0:
        #         change_clear = False
        #     else:
        #         change_clear = True

        # map_data = self.blackboard.get("/paf/hero/mapping/init_data")
        # map = Map.from_ros_msg(map_data)
        # checks if the left lane of the car is free,
        # otherwise pause lane change
        # tree = map.build_tree(mapping_common.map.lane_free_filter())

        change_clear = False
        # if map.entities and tree.is_lane_free(
        #    right_lane=False, lane_length=22.5, lane_transform=-5.0
        # ):
        if True:
            self.counter_lanefree += 1
            if self.counter_lanefree > 1:
                # rospy.loginfo("WAIT: Lane Change is free!")
                change_clear = True
        else:
            # rospy.loginfo("Lane Change blocked slowing down")
            change_clear = False
            self.counter_lanefree = 0

        rospy.loginfo(f"Lane Change wait counter: {self.counter_lanefree}")

        if not change_clear:
            rospy.loginfo("Lane Change Wait: blocked")
            self.curr_behavior_pub.publish(bs.lc_wait.name)
            return py_trees.common.Status.RUNNING
        else:
            rospy.loginfo("Lane Change Wait: Change clear")
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
            Whenever your behaviour switches to a non-running state.
           - SUCCESS || FAILURE : your behaviour's work cycle has finished
           - INVALID : a higher priority branch has interrupted, or shutting
           down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class Enter(py_trees.behaviour.Behaviour):
    """
    This behavior inititates the lane change and waits until the
    lane change is finished.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the target speed publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This prints a state status message and changes the driving speed for
        the lane change.
        """
        rospy.loginfo("Lane Change: Enter next Lane")
        self.curr_behavior_pub.publish(bs.lc_enter_init.name)

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Continues driving through the lane change until the vehicle gets
        close enough to the next global way point.
        :return: py_trees.common.Status.RUNNING, if too far from intersection
                 py_trees.common.Status.SUCCESS, if stopped in front of inter-
                 section or entered the intersection
                 py_trees.common.Status.FAILURE, if no next path point can be
                 detected.
        """
        next_waypoint_msg = self.blackboard.get("/paf/hero/lane_change_distance")

        if next_waypoint_msg is None:
            return py_trees.common.Status.FAILURE
        if next_waypoint_msg.distance < 5:
            rospy.loginfo("Lane Change Enter: Drive on the next lane!")
            return py_trees.common.Status.RUNNING
        else:
            return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
          - SUCCESS || FAILURE : your behaviour's work cycle has finished
          - INVALID : a higher priority branch has interrupted, or shutting
          down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )


class Leave(py_trees.behaviour.Behaviour):
    """
    This behaviour defines the leaf of this subtree, if this behavior is
    reached, the vehicle has finished the lane change.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. Other one-time initialisation
        requirements should be met via the setup() method.
        :param name: name of the behaviour
        """
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and the current behavior publisher.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as the set up is successful.
        """
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
            The first time your behaviour is ticked and anytime the status is
            not RUNNING thereafter.
        What to do here?
            Any initialisation you need before putting your behaviour to work.
        This prints a state status message and changes the driving speed to
        the street speed limit.
        """
        rospy.loginfo("Lane Change Finished")

        self.curr_behavior_pub.publish(bs.lc_exit.name)
        return True

    def update(self):
        """
        When is this called?
            Every time your behaviour is ticked.
        What to do here?
           - Triggering, checking, monitoring. Anything...but do not block!
           - Set a feedback message
           - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]
        Abort this subtree
        :return: py_trees.common.Status.FAILURE, to exit this subtree
        """
        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        When is this called?
           Whenever your behaviour switches to a non-running state.
          - SUCCESS || FAILURE : your behaviour's work cycle has finished
          - INVALID : a higher priority branch has interrupted, or shutting
          down
        writes a status message to the console when the behaviour terminates
        :param new_status: new state after this one is terminated
        """
        self.logger.debug(
            "  %s [Foo::terminate().terminate()][%s->%s]"
            % (self.name, self.status, new_status)
        )

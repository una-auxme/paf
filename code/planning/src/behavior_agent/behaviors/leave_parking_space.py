from typing import Optional
import py_trees
import rospy
from std_msgs.msg import String
import shapely


from . import behavior_speed as bs
from .stop_mark_service_utils import (
    create_stop_marks_proxy,
    update_stop_marks,
    get_global_hero_transform,
)
from .topics2blackboard import BLACKBOARD_MAP_ID
from .debug_markers import add_debug_marker, add_debug_entry, debug_status

import mapping_common.map
from mapping_common.map import Map, LaneFreeState
from mapping_common.markers import debug_marker

from std_msgs.msg import Float32

UNPARKING_MARKER_COLOR = (219 / 255, 255 / 255, 0.0, 1.0)


class LeaveParkingSpace(py_trees.behaviour.Behaviour):
    """
    This behavior is triggered in the beginning when the vehicle needs
    to leave the parking space.
    """

    def __init__(self, name):
        """
        Minimal one-time initialisation. A good rule of thumb is to only
        include the initialisation relevant for being able to insert this
        behaviour in a tree for offline rendering to dot graphs.

         :param name: name of the behaviour
        """
        super(LeaveParkingSpace, self).__init__(name)
        rospy.loginfo("LeaveParkingSpace started")
        self.finished = False
        self.stop_proxy = create_stop_marks_proxy()

    def setup(self, timeout):
        """
        Delayed one-time initialisation that would otherwise interfere with
        offline rendering of this behaviour in a tree to dot graph or
        validation of the behaviour's configuration.

        This initializes the blackboard to be able to access data written to it
        by the ROS topics and gathers the time to check how much time has
        passed.
        :param timeout: an initial timeout to see if the tree generation is
        successful
        :return: True, as there is nothing to set up.
        """
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        return True

    def initialise(self):
        """
        When is this called?
        The first time your behaviour is ticked and anytime the status is not
        RUNNING thereafter.

        What to do here?
            Any initialisation you need before putting your behaviour to work.
        Get initial position to check how far vehicle has moved during
        execution
        """
        self.added_stop: bool = False

    def add_initial_stop(self):
        """Add the initial stop mark"""
        # Just an empty map
        map = Map()
        # We just the lane free function to create the shape for our stopmarker
        tree = map.build_tree(mapping_common.map.lane_free_filter())
        _, mask = tree.is_lane_free(
            right_lane=False,
            lane_length=20.0,
            lane_transform=10.0,
            check_method="rectangle",
            reduce_lane=0.5,
        )
        if not isinstance(mask, shapely.Polygon):
            rospy.logfatal("Lanemask is not a polygon.")
        else:
            update_stop_marks(
                self.stop_proxy,
                id=self.name,
                reason="lane blocked",
                is_global=False,
                marks=[mask],
            )

    def update(self):
        """
        When is this called?
        Every time your behaviour is ticked.

        pose:
            position:
                x: 294.43757083094295
                y: -1614.961812061094
                z: 211.1994649671884
        What to do here?
            - Triggering, checking, monitoring. Anything...but do not block!
            - Set a feedback message
            - return a py_trees.common.Status.[RUNNING, SUCCESS, FAILURE]

        This behaviour runs until the agent has left the parking space.
        This is checked by calculating the euclidian distance thath the agent
        has moved since the start

        :return: py_trees.common.Status.RUNNING, while the agent is leaving
                                            the parking space
                 py_trees.common.Status.SUCCESS, never to continue with
                                            intersection
                 py_trees.common.Status.FAILURE, if not in parking
                 lane
        """

        if not self.finished:
            acc_speed: Optional[Float32] = self.blackboard.get("/paf/hero/acc_velocity")
            map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
            hero_transform = get_global_hero_transform()
            trajectory = self.blackboard.get("/paf/hero/trajectory_local")

            if (
                acc_speed is not None
                and map is not None
                and hero_transform is not None
                and trajectory is not None
            ):
                if not self.added_stop:
                    self.add_initial_stop()
                    self.added_stop = True

                # checks if the left lane of the car is free,
                tree = map.build_tree(mapping_common.map.lane_free_filter())
                state, mask = tree.is_lane_free(
                    right_lane=False,
                    lane_length=20,
                    lane_transform=-10,
                    check_method="lanemarking",
                    reduce_lane=0.5,
                )
                if mask is not None:
                    add_debug_marker(debug_marker(mask, color=UNPARKING_MARKER_COLOR))
                add_debug_entry(self.name, f"Lane state: {state.name}")
                if state is LaneFreeState.FREE:
                    self.curr_behavior_pub.publish(bs.parking.name)
                    update_stop_marks(
                        self.stop_proxy,
                        id=self.name,
                        reason="lane not blocked",
                        is_global=False,
                        marks=[],
                    )
                    self.finished = True
                    return debug_status(
                        self.name,
                        py_trees.common.Status.FAILURE,
                        "Removed stopmark, finished unparking",
                    )

                return debug_status(
                    self.name,
                    py_trees.common.Status.RUNNING,
                    "Waiting for free lane",
                )

            return debug_status(
                self.name,
                py_trees.common.Status.FAILURE,
                f"Missing data (False==None): "
                f"acc_speed: {acc_speed is not None}, "
                f"map: {map is not None}, "
                f"hero_transform: {hero_transform is not None}, "
                f"trajectory: {trajectory is not None}",
            )

        return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        pass

from typing import Optional
import py_trees
from py_trees.common import Status
import shapely

from rclpy.client import Client
from rclpy.publisher import Publisher

from std_msgs.msg import String, Float32

import mapping_common.map
from mapping_common.map import Map, LaneFreeState
from mapping_common.markers import debug_marker
from mapping_common.transform import Point2
from planning.behavior_agent.blackboard_utils import Blackboard

from . import behavior_names as bs
from .stop_mark_service_utils import (
    update_stop_marks,
    get_global_hero_transform,
)
from .topics2blackboard import BLACKBOARD_MAP_ID
from .debug_markers import add_debug_marker, add_debug_entry, debug_status
from . import get_logger

UNPARKING_MARKER_COLOR = (219 / 255, 255 / 255, 0.0, 1.0)


class LeaveParkingSpace(py_trees.behaviour.Behaviour):
    """
    This behavior is triggered in the beginning when the vehicle needs
    to leave the parking space.
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
        self.finished = False
        get_logger().info("LeaveParkingSpace started")

    def setup(self, **kwargs):
        self.blackboard = Blackboard()

    def initialise(self):
        self.added_stop: bool = False
        self.init_position: Optional[Point2] = None

    def add_initial_stop(self):
        """Add the initial stop mark"""
        # Just an empty map
        map = Map()
        # We just use the lane free function to create the shape for our stopmarker
        tree = map.build_tree(mapping_common.map.lane_free_filter())
        _, mask = tree.is_lane_free(
            right_lane=False,
            lane_length=20.0,
            lane_transform=10.0,
            check_method="rectangle",
            reduce_lane=0.5,
        )
        if not isinstance(mask, shapely.Polygon):
            get_logger().fatal("Lanemask is not a polygon.")
        else:
            update_stop_marks(
                self.stop_client,
                id=self.name,
                reason="lane blocked",
                is_global=False,
                marks=[mask],
            )

    def update(self):
        """
        This behavior runs until the lane is free to leave the parking space.

        :return: py_trees.common.Status.RUNNING, while the agent is
                    waiting for the lane to be free
                 py_trees.common.Status.SUCCESS, never to continue with
                    intersection
                 py_trees.common.Status.FAILURE, if data is not yet available
                    or self.finished
        """

        if not self.finished:
            self.curr_behavior_pub.publish(String(data=bs.parking.name))
            acc_speed: Optional[Float32] = self.blackboard.try_get(
                "/paf/hero/acc_velocity"
            )
            map: Optional[Map] = self.blackboard.try_get(BLACKBOARD_MAP_ID)
            hero_transform = get_global_hero_transform()
            trajectory = self.blackboard.try_get("/paf/hero/trajectory_local")

            if (
                acc_speed is not None
                and map is not None
                and hero_transform is not None
                and trajectory is not None
            ):
                if not self.added_stop:
                    self.add_initial_stop()
                    self.added_stop = True
                if self.init_position is None:
                    self.init_position = hero_transform.translation().point()

                # checks if the left lane of the car is free,
                tree = map.build_tree(mapping_common.map.lane_free_filter())
                state, mask = tree.is_lane_free(
                    right_lane=False,
                    lane_length=25,
                    lane_transform=-15,
                    check_method="fallback",
                    reduce_lane=0.5,
                )
                if mask is not None:
                    add_debug_marker(debug_marker(mask, color=UNPARKING_MARKER_COLOR))
                add_debug_entry(self.name, f"Lane state: {state.name}")
                if (
                    state is LaneFreeState.FREE
                    or hero_transform.translation()
                    .point()
                    .distance_to(self.init_position)
                    > 5.0
                ):
                    # Either the lane is free or
                    # we started moving away from out start point
                    update_stop_marks(
                        self.stop_client,
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
        if new_status is Status.FAILURE or new_status is Status.INVALID:
            update_stop_marks(
                self.stop_client,
                id=self.name,
                reason="unparking terminated",
                is_global=False,
                marks=[],
            )

import py_trees
from py_trees.common import Status
from typing import Optional, Tuple, Union
from std_msgs.msg import String, Float32

import rospy

import mapping_common.map
import mapping_common.mask
import mapping_common.entity
from mapping_common.map import Map, MapTree, LaneFreeState
from mapping_common.entity import ShapelyEntity, Entity, StopMark
from mapping_common.markers import debug_marker
from mapping_common.transform import Transform2D, Vector2, Point2
import shapely
from mapping_common.entity import FlagFilter, Car
from visualization_msgs.msg import MarkerArray
from planning.srv import OvertakeStatusResponse

from . import behavior_names as bs
from .topics2blackboard import BLACKBOARD_MAP_ID
from .debug_markers import add_debug_marker, debug_status, add_debug_entry
from .overtake_service_utils import (
    create_start_overtake_proxy,
    create_end_overtake_proxy,
    create_overtake_status_proxy,
    request_start_overtake,
    request_end_overtake,
    request_overtake_status,
)
from .stop_mark_service_utils import (
    create_stop_marks_proxy,
    update_stop_marks,
)

from local_planner.utils import (
    TARGET_DISTANCE_TO_STOP_OVERTAKE,
)

OVERTAKE_MARKER_COLOR = (17 / 255, 232 / 255, 35 / 255, 1.0)

OVERTAKE_SPACE_STOPMARKS_ID = "overtake_space"


def set_space_stop_mark(proxy: rospy.ServiceProxy, obstacle: Entity):
    reason = "Obstacle: overtake space"
    transform = (
        Transform2D.new_translation(Vector2.backward() * 3.0) * obstacle.transform
    )
    mark = StopMark(
        reason=reason,
        confidence=1.0,
        priority=1.0,
        shape=obstacle.shape,
        transform=transform,
    )
    update_stop_marks(
        proxy,
        id=OVERTAKE_SPACE_STOPMARKS_ID,
        reason=reason,
        is_global=False,
        marks=[mark],
    )


def unset_space_stop_mark(proxy: rospy.ServiceProxy):
    update_stop_marks(
        proxy,
        id=OVERTAKE_SPACE_STOPMARKS_ID,
        reason="no obstacle",
        is_global=False,
        marks=[],
    )


"""
Source: https://github.com/ll7/psaf2
"""


def calculate_obstacle(
    behavior_name: str,
    tree: MapTree,
    blackboard: py_trees.blackboard.Blackboard,
    front_mask_size: float,
    trajectory_check_length: float,
    overlap_percent: float,
) -> Union[Optional[Tuple[ShapelyEntity, float]], py_trees.common.Status]:
    """Calculates if there is an obstacle in front

    Args:
        behavior_name (str): Name of the behavior using the function.
            Input self.name here
        tree (MapTree): Filtered map tree for querying entities
        blackboard (py_trees.blackboard.Blackboard): Blackboard for fetching data
        front_mask_size (float): Length of the static box collision mask in front
        trajectory_check_length (float): Length of the trajectory collision mask
        overlap_percent (float):
            How much of an entity has to be inside the collision mask

    Returns:
        Union[Optional[Tuple[ShapelyEntity, float]], py_trees.common.Status]:
            - If the function fails to create a valid result: Returns a Status
            - If the function succeeds:
                - If there is an obstacle: Returns the entity and
                  its distance to the hero
                - No obstacle: None
    """
    # data preparation
    trajectory = blackboard.get("/paf/hero/trajectory_local")
    if trajectory is None:
        return debug_status(behavior_name, Status.FAILURE, "trajectory_local is None")

    hero: Optional[Entity] = tree.map.hero()
    if hero is None:
        return debug_status(
            behavior_name, py_trees.common.Status.FAILURE, "hero is None"
        )
    hero_width = hero.get_width()

    collision_masks = mapping_common.mask.build_lead_vehicle_collision_masks(
        Point2.new(hero.get_front_x(), 0.0),
        hero_width,
        trajectory,
        front_mask_size=front_mask_size,
        max_trajectory_check_length=trajectory_check_length,
    )
    if len(collision_masks) == 0:
        # We currently have no valid path to check for collisions.
        # -> cannot drive safely
        return debug_status(
            behavior_name, Status.FAILURE, "Unable to build collision mask!"
        )
    collision_mask = shapely.union_all(collision_masks)

    for mask in collision_masks:
        add_debug_marker(debug_marker(mask, color=OVERTAKE_MARKER_COLOR))

    entity_result = tree.get_nearest_entity(
        collision_mask, hero.to_shapely(), min_coverage_percent=overlap_percent
    )

    if entity_result is not None:
        entity, distance = entity_result
        add_debug_marker(
            debug_marker(entity.entity, color=OVERTAKE_MARKER_COLOR, scale_z=0.3)
        )
        return (entity, distance)
    else:
        return None


OVERTAKE_FREE = False


class Ahead(py_trees.behaviour.Behaviour):
    """
    This behaviour checks whether an object that needs to be overtaken is
    ahead
    """

    def __init__(self, name):
        super(Ahead, self).__init__(name)

    def setup(self, timeout):
        self.blackboard = py_trees.blackboard.Blackboard()
        self.ot_distance_pub = rospy.Publisher(
            "/paf/hero/" "overtake_distance", Float32, queue_size=1
        )
        self.marker_publisher = rospy.Publisher(
            "/paf/hero/" "overtake/debug_markers", MarkerArray, queue_size=1
        )
        self.stop_proxy = create_stop_marks_proxy()
        return True

    def initialise(self):
        # Counter for detecting overtake situation
        self.counter_overtake = 0
        self.old_obstacle_distance = 200
        unset_space_stop_mark(self.stop_proxy)
        return True

    def update(self):
        """
        Gets the current distance and speed to object in front.
        Increases a counter to overtake if there is a obstacle.
        :return: py_trees.common.Status.SUCCESS, if the counter crosses a
                 certain threshold.
                 py_trees.common.Status.FAILURE, if there is nothing
                 to overtake.
        """

        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Map is None"
            )
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        obstacle = calculate_obstacle(
            self.name,
            tree,
            self.blackboard,
            front_mask_size=0.0,
            trajectory_check_length=18.0,
            overlap_percent=0.35,
        )
        if isinstance(obstacle, py_trees.common.Status):
            return obstacle
        if obstacle is None:
            return debug_status(self.name, Status.FAILURE, "No obstacle")

        entity, obstacle_distance = obstacle
        entity = entity.entity

        if entity.motion is not None:
            obstacle_speed = entity.motion.linear_motion.length()
        else:
            obstacle_speed = 0

        # filter out false positives due to trajectory inconsistency
        if (
            entity.transform.translation().y() < -3.0
            or entity.transform.translation().y() > 3.0
        ):
            return debug_status(self.name, Status.FAILURE, "Obstacle filtered out")

        # increase counter when something is blocking the path
        if (
            (
                obstacle_speed > 1.7
                and obstacle_speed < 2.5
                and not isinstance(entity, Car)
            )
            or (obstacle_speed < 1.0)
        ) and obstacle_distance < 15:
            self.counter_overtake += 1
            add_debug_entry(self.name, f"Obstacle distance: {obstacle_distance}")
            add_debug_entry(self.name, f"Overtake counter: {self.counter_overtake}")
            if self.counter_overtake > 4:
                self.ot_distance_pub.publish(obstacle_distance)
                return debug_status(
                    self.name, Status.SUCCESS, "Overtake counter big enough"
                )
            self.old_obstacle_distance = obstacle_distance
            return debug_status(self.name, Status.RUNNING, "Wait for overtake counter")
        else:
            return debug_status(self.name, Status.FAILURE, "Obstacle distance too big")

    def terminate(self, new_status):
        pass


class Approach(py_trees.behaviour.Behaviour):
    """
    This behaviour is executed when the ego vehicle is in close proximity of
    an object which needs to be overtaken and
    overtake_ahead is triggered.
    It then handles the procedure for overtaking.
    """

    def __init__(self, name):
        super(Approach, self).__init__(name)
        rospy.loginfo("Init -> Overtake Behavior: Approach")

    def setup(self, timeout):
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.start_overtake_proxy = create_start_overtake_proxy()
        self.stop_proxy = create_stop_marks_proxy()
        return True

    def initialise(self):
        """
        This initializes the overtaking distance to a default value.
        """
        rospy.loginfo("Approaching Overtake")
        global OVERTAKE_FREE
        self.ot_distance = 30
        self.ot_counter = 0
        self.clear_distance = 50
        OVERTAKE_FREE = False

    def update(self):
        """
        Gets the current distance to overtake, the current oncoming lane status and the
        distance to collsion object. Slows down until stopped
        or oncoming clear.
        :return: py_trees.common.Status.RUNNING, while driving towards the obstacle
                 py_trees.common.Status.SUCCESS, if stopped behind the blocking
                 object or oncoming is free.
                 py_trees.common.Status.FAILURE, if the overtake is aborted
        """
        global OVERTAKE_FREE

        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Map is None"
            )
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        obstacle = calculate_obstacle(
            self.name,
            tree,
            self.blackboard,
            front_mask_size=1.5,
            trajectory_check_length=20.0,
            overlap_percent=0.5,
        )
        if isinstance(obstacle, py_trees.common.Status):
            return obstacle
        if obstacle is None:
            return debug_status(self.name, Status.FAILURE, "No obstacle")

        entity, self.ot_distance = obstacle
        entity = entity.entity

        if entity.motion is not None:
            obstacle_speed = entity.motion.linear_motion.length()
        else:
            obstacle_speed = 0

        add_debug_entry(self.name, f"Overtake distance: {self.ot_distance}")
        if obstacle_speed > 2.7:
            return debug_status(
                self.name, Status.FAILURE, "Overtake entity started moving"
            )

        # Only add stop space if the obstacle is standing
        if obstacle_speed < 1.0:
            set_space_stop_mark(self.stop_proxy, obstacle=entity)
        else:
            unset_space_stop_mark(self.stop_proxy)

        # slow down before overtake if blocked
        if self.ot_distance < 15.0:
            ot_free, ot_mask = tree.is_lane_free(
                right_lane=False,
                lane_length=self.clear_distance,
                lane_transform=10.0,
                check_method="fallback",
            )
            if isinstance(ot_mask, shapely.Polygon):
                add_debug_marker(debug_marker(ot_mask, color=OVERTAKE_MARKER_COLOR))
            add_debug_entry(self.name, f"Overtake free?: {ot_free.name}")
            if ot_free is LaneFreeState.FREE:
                self.ot_counter += 1
                # using a counter to account for inconsistencies
                if self.ot_counter > 3:
                    add_debug_entry(self.name, "Overtake is free not slowing down!")
                    request_start_overtake(
                        self.start_overtake_proxy, start_transition_length=5.0
                    )
                    self.curr_behavior_pub.publish(bs.ot_app_free.name)
                    # bool to skip Wait since oncoming is free
                    OVERTAKE_FREE = True
                    return debug_status(self.name, Status.SUCCESS, "Overtake free")
                else:
                    self.curr_behavior_pub.publish(bs.ot_app_blocked.name)
                    return debug_status(
                        self.name,
                        Status.RUNNING,
                        f"Overtake free count: {self.ot_counter}",
                    )
            else:
                self.ot_counter = 0
                add_debug_entry(
                    self.name, "Overtake Approach: oncoming blocked slowing down"
                )
                self.curr_behavior_pub.publish(bs.ot_app_blocked.name)

        elif self.ot_distance > 20.0:
            return debug_status(self.name, Status.FAILURE, "Obstacle too far away")

        if self.ot_distance < TARGET_DISTANCE_TO_STOP_OVERTAKE:
            self.curr_behavior_pub.publish(bs.ot_app_blocked.name)
            return debug_status(
                self.name, Status.SUCCESS, "Overtake Approach: stopping behind obstacle"
            )
        else:
            return debug_status(
                self.name,
                Status.RUNNING,
                f"Overtake Approach: still approaching obstacle, "
                f"distance: {self.ot_distance}",
            )

    def terminate(self, new_status):
        if new_status is Status.FAILURE or new_status is Status.INVALID:
            unset_space_stop_mark(self.stop_proxy)


class Wait(py_trees.behaviour.Behaviour):
    """
    This behavior handles the waiting in front of object,
    which is blocking the road.
    The Ego vehicle is waiting to get a clear path for overtaking.
    """

    def __init__(self, name):
        super(Wait, self).__init__(name)

    def setup(self, timeout):
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.start_overtake_proxy = create_start_overtake_proxy()
        self.blackboard = py_trees.blackboard.Blackboard()
        self.stop_proxy = create_stop_marks_proxy()
        return True

    def initialise(self):
        rospy.loginfo("Waiting for Overtake")
        # slightly less distance since we have already stopped
        self.clear_distance = 50
        self.ot_counter = 0
        self.ot_gone = 0
        return True

    def update(self):
        """
        Waits behind the road object until map function lane free check
        return True.

        :return: py_trees.common.Status.RUNNING, while is lane free returns False
                 py_trees.common.Status.SUCCESS, when lane free returns True
        """
        global OVERTAKE_FREE
        if OVERTAKE_FREE:
            self.curr_behavior_pub.publish(bs.ot_wait_free.name)
            return debug_status(
                self.name, py_trees.common.Status.SUCCESS, "Overtake free"
            )

        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Map is None"
            )
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        obstacle = calculate_obstacle(
            self.name,
            tree,
            self.blackboard,
            front_mask_size=1.0,
            trajectory_check_length=20.0,
            overlap_percent=0.5,
        )
        if isinstance(obstacle, py_trees.common.Status):
            return obstacle
        if obstacle is None:
            # using a counter to account for data inconsistencies
            self.ot_gone += 1
            if self.ot_gone > 3:
                return debug_status(
                    self.name, py_trees.common.Status.FAILURE, "Obstacle gone"
                )
            return py_trees.common.Status.RUNNING

        entity, distance = obstacle
        entity = entity.entity

        if entity.motion is not None:
            obstacle_speed = entity.motion.linear_motion.length()
        else:
            obstacle_speed = 0

        self.ot_gone = 0
        add_debug_entry(self.name, f"Obstacle speed: {obstacle_speed}")
        if obstacle_speed > 3.0:
            return debug_status(self.name, Status.FAILURE, "Obstacle started moving")

        # Only add stop space if the obstacle is standing
        if obstacle_speed < 1.0:
            set_space_stop_mark(self.stop_proxy, obstacle=entity)
        else:
            unset_space_stop_mark(self.stop_proxy)

        self.curr_behavior_pub.publish(bs.ot_wait_free.name)
        ot_free, ot_mask = tree.is_lane_free(
            right_lane=False,
            lane_length=self.clear_distance,
            lane_transform=10.0,
            check_method="fallback",
        )

        if isinstance(ot_mask, shapely.Polygon):
            add_debug_marker(debug_marker(ot_mask, color=OVERTAKE_MARKER_COLOR))
        add_debug_entry(self.name, f"Overtake free?: {ot_free.name}")
        if ot_free is LaneFreeState.FREE:
            self.ot_counter += 1
            if self.ot_counter > 3:
                self.curr_behavior_pub.publish(bs.ot_wait_free.name)
                request_start_overtake(
                    self.start_overtake_proxy, start_transition_length=0.0
                )
                return debug_status(self.name, Status.SUCCESS, "Overtake free")
            else:
                return debug_status(
                    self.name, Status.RUNNING, f"Overtake free count: {self.ot_counter}"
                )
        else:
            self.ot_counter = 0
            return debug_status(self.name, Status.RUNNING, "Overtake blocked")

    def terminate(self, new_status):
        if new_status is Status.FAILURE or new_status is Status.INVALID:
            unset_space_stop_mark(self.stop_proxy)


class Enter(py_trees.behaviour.Behaviour):
    """
    This behavior handles the switching to a new lane in the
    overtaking procedure.
    """

    def __init__(self, name):
        super(Enter, self).__init__(name)

    def setup(self, timeout):
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.overtake_status_proxy = create_overtake_status_proxy()
        self.stop_proxy = create_stop_marks_proxy()
        return True

    def initialise(self):
        """
        This prints a state status message and publishes the behavior to
        trigger the replanning
        """
        rospy.loginfo("Enter Overtake")
        self.curr_behavior_pub.publish(bs.ot_enter_init.name)

    def update(self):
        """
        Waits for the hero to enter the overtake.
        """
        unset_space_stop_mark(self.stop_proxy)
        status: OvertakeStatusResponse = request_overtake_status(
            self.overtake_status_proxy
        )

        if status.status == OvertakeStatusResponse.OVERTAKING:
            return debug_status(self.name, Status.SUCCESS, "Overtaking")
        elif status.status == OvertakeStatusResponse.OVERTAKE_QUEUED:
            return debug_status(
                self.name,
                Status.RUNNING,
                "Overtake queued. Waiting for OvertakeStatusResponse.OVERTAKING...",
            )
        elif status.status == OvertakeStatusResponse.NO_OVERTAKE:
            return debug_status(
                self.name, Status.FAILURE, "Abort: OvertakeStatusResponse.NO_OVERTAKE"
            )
        elif status.status == OvertakeStatusResponse.OVERTAKE_ENDING:
            return debug_status(
                self.name,
                Status.FAILURE,
                "Abort: OvertakeStatusResponse.OVERTAKE_ENDING",
            )
        else:
            return debug_status(
                self.name,
                Status.FAILURE,
                f"Abort: Unknown OvertakeStatus: {status.status}",
            )

    def terminate(self, new_status):
        pass


class Leave(py_trees.behaviour.Behaviour):
    """
    This behavior defines the leaf of this subtree, if this behavior is
    reached, the vehicle performed the overtake.
    """

    def __init__(self, name):
        super(Leave, self).__init__(name)

    def setup(self, timeout):
        self.curr_behavior_pub = rospy.Publisher(
            "/paf/hero/" "curr_behavior", String, queue_size=1
        )
        self.blackboard = py_trees.blackboard.Blackboard()
        self.overtake_status_proxy = create_overtake_status_proxy()
        self.end_overtake_proxy = create_end_overtake_proxy()
        return True

    def initialise(self):
        self.curr_behavior_pub.publish(bs.ot_leave.name)
        return True

    def update(self):
        """
        Abort this subtree, if overtake distance is big enough
        :return: py_trees.common.Status.FAILURE, to exit this subtree
        """
        map: Optional[Map] = self.blackboard.get(BLACKBOARD_MAP_ID)
        if map is None:
            return debug_status(
                self.name, py_trees.common.Status.FAILURE, "Map is None"
            )
        tree = map.build_tree(FlagFilter(is_collider=True, is_hero=False))

        status: OvertakeStatusResponse = request_overtake_status(
            self.overtake_status_proxy
        )

        if status.status == OvertakeStatusResponse.NO_OVERTAKE:
            return debug_status(
                self.name, Status.FAILURE, "OvertakeStatusResponse.NO_OVERTAKE"
            )

        if status.status == OvertakeStatusResponse.OVERTAKING:
            ot_free, ot_mask = tree.is_lane_free(
                right_lane=True,
                lane_length=15.0,
                lane_transform=7.5,
                check_method="lanemarking",
            )
            add_debug_entry(self.name, f"Right lane free?: {ot_free.name}")
            if isinstance(ot_mask, shapely.Polygon):
                add_debug_marker(debug_marker(ot_mask, color=OVERTAKE_MARKER_COLOR))
            if ot_free is LaneFreeState.FREE:
                request_end_overtake(self.end_overtake_proxy)
                return debug_status(
                    self.name,
                    Status.RUNNING,
                    "Right lane is free. Finishing overtake...",
                )

        if status.status == OvertakeStatusResponse.OVERTAKE_ENDING:
            return debug_status(
                self.name,
                Status.RUNNING,
                "Waiting until we are back on the normal trajectory",
            )

        return debug_status(self.name, Status.RUNNING, "Waiting for right lane free...")

    def terminate(self, new_status):
        pass

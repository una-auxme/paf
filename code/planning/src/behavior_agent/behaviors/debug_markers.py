from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass, field
import time

from mapping_common.markers import debug_marker, debug_marker_array
from mapping_common.transform import Vector2

import py_trees

import rospy
from visualization_msgs.msg import MarkerArray, Marker
from std_msgs.msg import String

MARKER_NAMESPACE: str = "behavior_tree"

DEBUG_MARKER_LIST_ID: str = "/debug/markers"
"""Blackboard: Contains a list of queued markers for visualization
"""
_marker_error_msg: str = (
    f"Blackboard entry {DEBUG_MARKER_LIST_ID} is not properly set up"
)
DEBUG_INFO_DICT_ID: str = "/debug/tree_info"
"""Blackboard: Contains a dict of key: *Behavior name* and value: *BehaviorDebugInfo*
"""
_info_error_msg = f"Blackboard entry {DEBUG_INFO_DICT_ID} is not properly set up"


def add_debug_marker(m: Marker):
    """Queues a marker for visualization.

    Can be used in any behavior that is called between the
    DebugMarkerBlackboardSetupBehavior and the
    DebugMarkerBlackboardPublishBehavior
    """
    blackboard = py_trees.blackboard.Blackboard()
    marker_list: Optional[List[Marker]] = blackboard.get(DEBUG_MARKER_LIST_ID)
    if marker_list is None:
        rospy.logwarn(_marker_error_msg)
        return

    marker_list.append(m)


def add_debug_entry(
    behavior_name: str,
    entry: str,
):
    """Adds a debug message entry for a behavior

    Args:
        behavior_name (str): Name of the behavior the message belongs to.
            Recommended: Inside the Behavior, use *self.name*
        entry (str): Debug entry
    """
    blackboard = py_trees.blackboard.Blackboard()
    info_dict: Optional[Dict] = blackboard.get(DEBUG_INFO_DICT_ID)
    if info_dict is None:
        rospy.logwarn(_info_error_msg)
        return

    if behavior_name in info_dict:
        info: BehaviorDebugInfo = info_dict[behavior_name]
        info.entries.append(entry)
    else:
        info = BehaviorDebugInfo(entries=[entry])
        info_dict[behavior_name] = info


def debug_status(
    behavior_name: str, status: py_trees.common.Status, reason: Optional[str] = None
) -> py_trees.common.Status:
    """Updates debug status information for a behavior

    Recommended usage: When returning a status from the behavior *update()* function,
    WRAP the status return with this function

    Args:
        behavior_name (str): Name of the behavior the status belongs to.
            Recommended: Inside the Behavior, use *self.name*
        status (py_trees.common.Status): New behavior status
        reason (Optional[str], optional): Why this status was entered. Defaults to None.

    Returns:
        py_trees.common.Status: _description_
    """
    blackboard = py_trees.blackboard.Blackboard()
    info_dict: Optional[Dict[str, BehaviorDebugInfo]] = blackboard.get(
        DEBUG_INFO_DICT_ID
    )
    if info_dict is None:
        rospy.logwarn(_info_error_msg)
        return status

    if behavior_name in info_dict:
        info: BehaviorDebugInfo = info_dict[behavior_name]
        info.status = (status, reason)
    else:
        info = BehaviorDebugInfo(status=(status, reason))
        info_dict[behavior_name] = info

    return status


@dataclass(init=False)
class BehaviorDebugInfo:
    """Status and debug entries for a behavior"""

    status: Optional[Tuple[py_trees.common.Status, Optional[str]]]
    """Tuple: (Status, Reason (optional))
    """
    entries: List[str] = field(default_factory=list)
    """List of debug messages
    """

    def __init__(
        self,
        status: Optional[Tuple[py_trees.common.Status, Optional[str]]] = None,
        entries: Optional[List[str]] = None,
    ):
        self.entries = [] if entries is None else entries
        self.status = status
        # Used for sorting the entries in the visualization
        self._sys_creation_time = time.time_ns()

    def to_string(self, name: str) -> str:
        status_str = "UNKNOWN"
        if self.status is not None:
            status, reason = self.status
            if reason is None:
                status_str = status.name
            else:
                status_str = f"{status.name}: {reason}"
        result: str = f"{name}: {status_str}"
        for e in self.entries:
            result += f"\n  - {e}"
        return result


class DebugMarkerBlackboardSetupBehavior(py_trees.Behaviour):
    """Sets up an empty list/dict for the *DEBUG_MARKER_LIST_ID* and
    *DEBUG_INFO_DICT_ID* in the blackboard
    """

    def __init__(self, *args, **kwargs):
        super().__init__(type(self).__name__, *args, **kwargs)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.blackboard.set(DEBUG_MARKER_LIST_ID, [])
        self.blackboard.set(DEBUG_INFO_DICT_ID, {})
        return py_trees.common.Status.SUCCESS


class DebugMarkerBlackboardPublishBehavior(py_trees.Behaviour):
    """Reads the markers and debug entries from *DEBUG_MARKER_LIST_ID* and
    *DEBUG_INFO_DICT_ID* inside the blackboard and publishes them into ROS/RViz
    """

    def __init__(self, *args, **kwargs):
        super().__init__(type(self).__name__, *args, **kwargs)
        self.blackboard = py_trees.blackboard.Blackboard()
        self.marker_publisher = rospy.Publisher(
            "/paf/hero/behavior_tree/debug_markers", MarkerArray, queue_size=1
        )
        self.info_publisher = rospy.Publisher(
            "/paf/hero/behavior_tree/info_marker", Marker, queue_size=1
        )

    def update(self):
        marker_list: Optional[List[Marker]] = self.blackboard.get(DEBUG_MARKER_LIST_ID)
        if marker_list is None:
            rospy.logwarn(_marker_error_msg)
        else:
            marker_array = debug_marker_array(MARKER_NAMESPACE, marker_list)
            self.marker_publisher.publish(marker_array)

        info_dict: Optional[Dict[str, BehaviorDebugInfo]] = self.blackboard.get(
            DEBUG_INFO_DICT_ID
        )

        current_behavior_topic_msg: Optional[String] = self.blackboard.get(
            "/paf/hero/curr_behavior"
        )
        current_behavior_topic = (
            "None"
            if current_behavior_topic_msg is None
            else current_behavior_topic_msg.data
        )
        info_text = (
            f"Behavior from curr_behavior topic: {current_behavior_topic}\n"
            f"Behavior Tree Overview:"
        )
        if info_dict is None:
            rospy.logwarn(_info_error_msg)
        else:
            info_items = list(info_dict.items())
            info_items.sort(key=lambda i: i[1]._sys_creation_time)
            for name, info in info_items:
                info_text += f"\n{info.to_string(name)}"
        info_marker = debug_marker(
            info_text,
            position_z=-2.0,
            offset=Vector2.new(-2.0, 0.0),
            color=(1.0, 1.0, 1.0, 1.0),
        )
        self.info_publisher.publish(info_marker)

        return py_trees.common.Status.SUCCESS

from typing import List, Optional, Dict, Tuple
from dataclasses import dataclass, field

from mapping_common.markers import debug_marker, debug_marker_array
from mapping_common.transform import Vector2

import py_trees

import rospy
from visualization_msgs.msg import MarkerArray, Marker

MARKER_NAMESPACE: str = "behavior_tree"

DEBUG_MARKER_LIST_ID: str = "/debug/markers"
_marker_error_msg: str = (
    f"Blackboard entry {DEBUG_MARKER_LIST_ID} is not properly set up"
)
DEBUG_INFO_DICT_ID: str = "/debug/tree_info"
_info_error_msg = f"Blackboard entry {DEBUG_INFO_DICT_ID} is not properly set up"


def add_debug_marker(m: Marker):
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


@dataclass
class BehaviorDebugInfo:
    status: Optional[Tuple[py_trees.common.Status, Optional[str]]] = None
    """Tuple: (Status, Reason (optional))
    """
    entries: List[str] = field(default_factory=list)

    def to_string(self, name: str) -> str:
        status_str = "???"
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

    def __init__(self, *args, **kwargs):
        super().__init__(type(self).__name__, *args, **kwargs)
        self.blackboard = py_trees.blackboard.Blackboard()

    def update(self):
        self.blackboard.set(DEBUG_MARKER_LIST_ID, [])
        self.blackboard.set(DEBUG_INFO_DICT_ID, {})
        return py_trees.common.Status.SUCCESS


class DebugMarkerBlackboardPublishBehavior(py_trees.Behaviour):

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

        info_text = "Behavior Tree Overview:"
        if info_dict is None:
            rospy.logwarn(_info_error_msg)
        else:
            for name, info in info_dict.items():
                info_text += f"\n{info.to_string(name)}"
        info_marker = debug_marker(
            info_text,
            position_z=-2.0,
            offset=Vector2.new(-2.0, 0.0),
            color=(1.0, 1.0, 1.0, 1.0),
        )
        self.info_publisher.publish(info_marker)

        return py_trees.common.Status.SUCCESS

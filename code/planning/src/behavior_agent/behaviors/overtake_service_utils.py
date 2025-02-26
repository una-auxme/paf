import rospy
from typing import Optional
from py_trees.blackboard import Blackboard

from planning.srv import (
    StartOvertake,
    EndOvertake,
    OvertakeStatus,
    StartOvertakeRequest,
    EndOvertakeRequest,
    StartOvertakeResponse,
    EndOvertakeResponse,
    OvertakeStatusResponse,
)

import mapping_common.map
from mapping_common.transform import Point2, Transform2D

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32


def create_start_overtake_proxy() -> rospy.ServiceProxy:
    service = rospy.ServiceProxy(
        "/paf/hero/motion_planning/start_overtake", StartOvertake
    )
    service.wait_for_service()
    return service


def create_end_overtake_proxy() -> rospy.ServiceProxy:
    service = rospy.ServiceProxy("/paf/hero/motion_planning/end_overtake", EndOvertake)
    service.wait_for_service()
    return service


def create_overtake_status_proxy() -> rospy.ServiceProxy:
    service = rospy.ServiceProxy(
        "/paf/hero/motion_planning/overtake_status", OvertakeStatus
    )
    service.wait_for_service()
    return service


def _get_global_hero_transform() -> Transform2D:
    blackboard = Blackboard()
    current_pos: PoseStamped = blackboard.get("/paf/hero/current_pos")
    current_heading: Float32 = blackboard.get("/paf/hero/current_heading")
    hero_transform = mapping_common.map.build_global_hero_transform(
        current_pos.pose.position.x,
        current_pos.pose.position.y,
        current_heading.data,
    )
    return hero_transform


def request_start_overtake(
    proxy: rospy.ServiceProxy,
    local_start_pos: Optional[Point2] = None,
    local_end_pos: Optional[Point2] = None,
    offset: float = 2.5,
    start_transition_length: float = 2.0,
    end_transition_length: float = 2.0,
) -> StartOvertakeResponse:
    req = StartOvertakeRequest(
        offset=offset,
        start_transition_length=start_transition_length,
        end_transition_length=end_transition_length,
    )
    hero_transform = _get_global_hero_transform()
    if local_start_pos is not None:
        global_start_pos: Point2 = hero_transform * local_start_pos
        req.has_start_pos = True
        req.start_pos = global_start_pos.to_ros_msg()
    if local_end_pos is not None:
        global_end_pos: Point2 = hero_transform * local_end_pos
        req.has_end_pos = True
        req.end_pos = global_end_pos.to_ros_msg()

    return proxy(req)


def request_end_overtake(
    proxy: rospy.ServiceProxy,
    local_end_pos: Optional[Point2] = None,
    end_transition_length: float = 2.0,
) -> EndOvertakeResponse:
    req = EndOvertakeRequest(end_transition_length=end_transition_length)
    if local_end_pos is not None:
        hero_transform = _get_global_hero_transform()
        global_end_pos: Point2 = hero_transform * local_end_pos
        req.has_end_pos = True
        req.end_pos = global_end_pos.to_ros_msg()

    return proxy(req)


def request_overtake_status(proxy: rospy.ServiceProxy) -> OvertakeStatusResponse:
    return proxy()

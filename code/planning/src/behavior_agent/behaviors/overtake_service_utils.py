import rospy
from typing import Optional
from py_trees.blackboard import Blackboard
from geometry_msgs.msg import PoseStamped

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


def get_global_hero_transform() -> Optional[Transform2D]:
    """Returns the current hero transform based on data in the blackboard

    Returns:
        Optional[Transform2D]: None if the required data is not yet available
    """
    blackboard = Blackboard()
    current_pos: Optional[PoseStamped] = blackboard.get("/paf/hero/current_pos")
    current_heading: Optional[Float32] = blackboard.get("/paf/hero/current_heading")
    if current_pos is None or current_heading is None:
        return None
    hero_transform = mapping_common.map.build_global_hero_transform(
        current_pos.pose.position.x,
        current_pos.pose.position.y,
        current_heading.data,
    )
    return hero_transform


def _get_global_hero_transform() -> Optional[Transform2D]:
    """Deprecated. Use get_global_hero_transform instead"""
    return get_global_hero_transform()


def request_start_overtake(
    proxy: rospy.ServiceProxy,
    local_start_pos: Optional[Point2] = None,
    local_end_pos: Optional[Point2] = None,
    offset: float = 2.75,
    start_transition_length: float = 2.0,
    end_transition_length: float = 2.0,
) -> Optional[StartOvertakeResponse]:
    """Convenience function for the StartOvertake service

    Queues an overtake inside the motion planning or
    updates an existing one with the new settings

    Note that transition lengths are cropped from the overtake
    between start and end point.

    Note that the local positions are internally converted into
    global positions and then sent to the service.

    Args:
        proxy (rospy.ServiceProxy): Service proxy to use for sending the request.
            Use create_start_overtake_proxy() to create it.
        local_start_pos (Optional[Point2], optional):
            If set, the overtake starts at start_pos.
            If None, the overtake starts immediately.
            Defaults to None.
        local_end_pos (Optional[Point2], optional):
            If set, the overtake ends at end_pos.
            If None, the hero will stay in the overtake indefinitely
            until an EndOvertake request is received.
            Defaults to None.
        offset (float, optional): How far the overtake trajectory
            should be offset from the base trajectory in m.
            If negative, the overtake will be offset to the right.
            Defaults to 2.5.
        start_transition_length (float, optional): Transition length towards
            the overtake.
            Defaults to 2.0.
        end_transition_length (float, optional): Transition length away from
            the overtake.
            Defaults to 2.0.

    Returns:
        Optional[StartOvertakeResponse]: _description_
    """
    req = StartOvertakeRequest(
        offset=offset,
        start_transition_length=start_transition_length,
        end_transition_length=end_transition_length,
    )
    hero_transform = get_global_hero_transform()
    if hero_transform is None:
        return None
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
) -> Optional[EndOvertakeResponse]:
    """Convenience function for the EndOvertake service

    Ends the currently running overtake / aborts a queued one.
    Does nothing if no overtake is running/queued.

    Note that transition lengths are cropped from the overtake
    between start and end point.

    Note that the local positions are internally converted into
    global positions and then sent to the service.

    Args:
        proxy (rospy.ServiceProxy): Service proxy to use for sending the request.
            Use create_end_overtake_proxy() to create it.
        local_end_pos (Optional[Point2], optional):
            If set, the overtake ends at end_pos.
            If None, the hero will leave the overtake immediately.
            Defaults to None.
        end_transition_length (float, optional): Transition length away from
            the overtake.
            Defaults to 2.0.

    Returns:
        Optional[EndOvertakeResponse]
    """
    req = EndOvertakeRequest(end_transition_length=end_transition_length)
    if local_end_pos is not None:
        hero_transform = get_global_hero_transform()
        if hero_transform is None:
            return None
        global_end_pos: Point2 = hero_transform * local_end_pos
        req.has_end_pos = True
        req.end_pos = global_end_pos.to_ros_msg()

    return proxy(req)


def request_overtake_status(proxy: rospy.ServiceProxy) -> OvertakeStatusResponse:
    """Convenience function for the OvertakeStatus service

    Args:
        proxy (rospy.ServiceProxy): Service proxy to use for sending the request.
            Use create_overtake_status_proxy() to create it.

    Returns:
        OvertakeStatusResponse: Status response from the service.
            Look in planning/srv/OvertakeStatus.srv for it's definition
    """
    return proxy()

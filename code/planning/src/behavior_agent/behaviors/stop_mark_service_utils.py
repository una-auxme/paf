import rospy
from typing import Union, List, Optional
from copy import deepcopy

import shapely

from mapping.srv import UpdateStopMarks, UpdateStopMarksRequest, UpdateStopMarksResponse

from mapping_common.transform import Transform2D
from mapping_common.entity import StopMark, Flags
from mapping_common.shape import Shape2D, Polygon

from .overtake_service_utils import get_global_hero_transform


def create_stop_marks_proxy() -> rospy.ServiceProxy:
    service = rospy.ServiceProxy("/paf/hero/mapping/update_stop_marks", UpdateStopMarks)
    service.wait_for_service()
    return service


def update_stop_marks(
    proxy: rospy.ServiceProxy,
    id: str,
    reason: str,
    is_global: bool = False,
    marks: Optional[List[Union[StopMark, Shape2D, shapely.Polygon]]] = None,
    delete_all_others: bool = False,
) -> Optional[UpdateStopMarksResponse]:
    """Convenience function for the UpdateStopMarks service
    of the mapping_data_integration.

    Place StopMarks into the map that act as virtual obstacles.

    To clear a set of StopMarks with *id*, pass an empty list to *marks*.

    The service definition can be found in mapping/srv/UpdateStopMarks.srv.

    Args:
        proxy (rospy.ServiceProxy): Service proxy to use for sending the request.
            Use create_stop_marks_proxy() to create it.
        id (str): Identifier of this set of StopMarks
        reason (str): Reason for this set of StopMarks
        is_global (bool, optional): If False, the coordinates of *marks*
            are converted into global coordinates before sending the request.
            Defaults to False.
        marks (List[Union[StopMark, Shape2D, shapely.Polygon]], optional):
            List of StopMarks to add to the map.
            Defaults to [].
        delete_all_others (bool, optional): If True, deletes all StopMarks from the map.
            Defaults to False.

    Returns:
        Optional[UpdateStopMarksResponse]
    """
    if marks is None:
        marks = []
    if not is_global:
        hero_transform = get_global_hero_transform()
        if hero_transform is None:
            return None

    global_marks: List[StopMark] = []
    for mark in marks:
        if isinstance(mark, StopMark):
            e = deepcopy(mark)
            if len(e.reason) == 0:
                e.reason = reason
        elif isinstance(mark, Shape2D):
            shape = deepcopy(mark)
            transform = shape.offset
            shape.offset = Transform2D.identity()
            e = StopMark(
                reason=reason,
                confidence=1.0,
                priority=1.0,
                shape=shape,
                transform=transform,
            )
        elif isinstance(mark, shapely.Polygon):
            shape = Polygon.from_shapely(mark, make_centered=True)
            transform = shape.offset
            shape.offset = Transform2D.identity()
            e = StopMark(
                reason=reason,
                confidence=1.0,
                priority=1.0,
                shape=shape,
                transform=transform,
            )
        else:
            rospy.logerr(f"Unsupported stop mark type: ${type(mark)}")
            continue

        if not is_global:
            e.transform = hero_transform * e.transform
        e.flags = Flags(is_stopmark=True)
        global_marks.append(e)

    ros_entities = [e.to_ros_msg() for e in global_marks]

    req = UpdateStopMarksRequest(
        id=id, delete_all_others=delete_all_others, marks=ros_entities
    )

    return proxy(req)

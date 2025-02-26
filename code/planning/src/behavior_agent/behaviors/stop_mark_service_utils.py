import rospy
from typing import Union, List

import shapely

from mapping.srv import UpdateStopMarks, UpdateStopMarksRequest, UpdateStopMarksResponse

from mapping_common.entity import StopMark
from mapping_common.shape import Shape2D

from .overtake_service_utils import _get_global_hero_transform


def create_stop_marks_proxy() -> rospy.ServiceProxy:
    service = rospy.ServiceProxy("/paf/hero/mapping/update_stop_marks", UpdateStopMarks)
    service.wait_for_service()
    return service


def update_stop_marks(
    proxy: rospy.ServiceProxy,
    reason: str,
    local_marks: List[Union[StopMark, Shape2D, shapely.Polygon]] = [],
    delete_all_others: bool = False,
) -> UpdateStopMarksResponse:
    hero_transform = _get_global_hero_transform()
    global_marks: List[StopMark] = []
    for mark in local_marks:
        # TODO
        pass

    req = UpdateStopMarksRequest(
        reason=reason, delete_all_others=delete_all_others, marks=global_marks
    )

    return proxy(req)

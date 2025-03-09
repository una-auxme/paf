from typing import Optional
from py_trees.blackboard import Blackboard

from nav_msgs.msg import Path
from perception.msg import Waypoint

import mapping_common.mask
from mapping_common.transform import Point2, Vector2

from .overtake_service_utils import get_global_hero_transform


def calculate_waypoint_distance(
    blackboard: Blackboard, waypoint: Waypoint, forward_offset: float = 0.0
) -> Optional[float]:
    """Calculates the distance of the hero to the waypoint

    Takes into account the trajectory_local.
    If the hero has already driven "over" the waypoint, the result will be <= 0.0

    Args:
        blackboard (Blackboard)
        waypoint (Waypoint): Waypoint to calculate the distance to

    Returns:
        Optional[float]: None, if information is missing in the blackboard
    """
    trajectory_local_msg: Optional[Path] = blackboard.get("/paf/hero/trajectory_local")
    if trajectory_local_msg is None:
        return None
    trajectory_local = mapping_common.mask.ros_path_to_line(trajectory_local_msg)

    stop_line_position = waypoint.position
    hero_transform = get_global_hero_transform()
    if hero_transform is None:
        return None
    local_pos: Point2 = (
        hero_transform.inverse()
        * Point2.new(stop_line_position.x, stop_line_position.y)
    ) + Vector2.forward() * forward_offset

    distance = trajectory_local.line_locate_point(local_pos.to_shapely())
    return distance

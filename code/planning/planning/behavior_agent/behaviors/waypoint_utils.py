from typing import Optional
from py_trees.blackboard import Blackboard

from nav_msgs.msg import Path
from perception.msg import Waypoint

import mapping_common.hero
import mapping_common.mask
from mapping_common.transform import Point2, Vector2

from .overtake_service_utils import get_global_hero_transform


def calculate_waypoint_distance(
    blackboard: Blackboard, waypoint: Waypoint, forward_offset: float = 0.0
) -> Optional[float]:
    """Calculates the distance of the hero(front) to the waypoint

    Takes into account the trajectory_local.
    If the hero has already driven "over" the waypoint, the result will be roughly 0.
    **IMPORTANT**: ROUGHLY 0. Recommended: check for <= 0.2

    Args:
        blackboard (Blackboard)
        waypoint (Waypoint): Waypoint to calculate the distance to

    Returns:
        Optional[float]: None, if information is missing in the blackboard
    """
    trajectory_local_msg: Optional[Path] = blackboard.get("/paf/hero/trajectory_local")
    if trajectory_local_msg is None:
        return None
    hero = mapping_common.hero.create_hero_entity()
    hero_front_x = hero.get_front_x()
    front_point = Point2.new(hero_front_x, 0.0)
    trajectory_local = mapping_common.mask.build_trajectory_from_start(
        trajectory_local=trajectory_local_msg, start_point=front_point
    )
    if trajectory_local is None:
        return None

    hero_transform = get_global_hero_transform()
    if hero_transform is None:
        return None
    local_pos: Point2 = (
        hero_transform.inverse() * Point2.new(waypoint.position.x, waypoint.position.y)
    ) + Vector2.forward() * forward_offset

    distance = trajectory_local.line_locate_point(local_pos.to_shapely())
    return distance

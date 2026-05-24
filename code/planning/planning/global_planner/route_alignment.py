"""Helpers for matching the ego pose to a global route."""

from __future__ import annotations

from math import hypot
from typing import Sequence


def find_route_alignment_index(
    agent_position: tuple[float, float],
    route_points: Sequence[tuple[float, float]],
    max_distance_m: float,
) -> int | None:
    """Return the nearest route index if it is within the accepted distance."""
    if not route_points:
        return None

    agent_x, agent_y = agent_position
    nearest_index = 0
    nearest_distance = float("inf")
    for index, (route_x, route_y) in enumerate(route_points):
        distance = hypot(agent_x - route_x, agent_y - route_y)
        if distance < nearest_distance:
            nearest_index = index
            nearest_distance = distance

    if nearest_distance > max_distance_m:
        return None
    return nearest_index

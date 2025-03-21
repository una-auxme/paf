from trajectory_generator_base import TrajectoryGenerator
from helper_functions import interpolate_route

import numpy as np
import math

from typing import List, Tuple

NAME = "sine"


class SineGenerator(TrajectoryGenerator):

    def __init__(self):
        super().__init__(NAME)

    def generate_trajectory(self, origin: Tuple[float, float]):
        origin_x, origin_y = origin
        startx = origin_x
        starty = origin_y + 50

        route: List[Tuple[float, float]] = []
        for y_offset in range(500):
            x = startx + math.sin(y_offset / 2) * 0.8  # 5 #2 # 40
            y = starty - y_offset
            route.append((x, y))

        return np.array(interpolate_route(route))

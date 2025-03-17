from trajectory_generator_base import TrajectoryGenerator
from helper_functions import interpolate_route

import numpy as np
import math

from typing import List, Tuple

NAME = "lanechange"


class LanechangeGenerator(TrajectoryGenerator):

    def __init__(self):
        super().__init__(NAME)

    def generate_trajectory(self, origin: Tuple[float, float]):
        origin_x, origin_y = origin
        startx = origin_x
        starty = origin_y

        lanechange_trajectory: List[Tuple[float, float]] = [
            (startx, starty),
            (startx - 0.5, starty - 10),
            (startx - 0.5, starty - 20),
            (startx - 0.4, starty - 21),
            (startx - 0.3, starty - 22),
            (startx - 0.2, starty - 23),
            (startx - 0.1, starty - 24),
            (startx, starty - 25),
            (startx + 0.1, starty - 26),
            (startx + 0.2, starty - 27),
            (startx + 0.3, starty - 28),
            (startx + 0.4, starty - 29),
            (startx + 0.5, starty - 30),
            (startx + 0.6, starty - 31),
            (startx + 0.7, starty - 32),
            (startx + 0.8, starty - 33),
            (startx + 0.9, starty - 34),
            (startx + 1.0, starty - 35),
            (startx + 1.0, starty - 50),
            (startx + 1.0, starty - 51),
            (startx + 0.9, starty - 52),
            (startx + 0.8, starty - 53),
            (startx + 0.7, starty - 54),
            (startx + 0.6, starty - 55),
            (startx + 0.5, starty - 56),
            (startx + 0.4, starty - 57),
            (startx + 0.3, starty - 58),
            (startx + 0.2, starty - 59),
            (startx + 0.1, starty - 60),
            (startx, starty - 61),
            (startx - 0.1, starty - 62),
            (startx - 0.2, starty - 63),
            (startx - 0.3, starty - 64),
            (startx - 0.4, starty - 65),
            (startx - 0.5, starty - 66),
            (startx - 0.5, starty - 100),
        ]

        return np.array(interpolate_route(lanechange_trajectory))

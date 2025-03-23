#!/usr/bin/env python

import ros_compatibility as roscomp
import numpy as np

from abc import ABC, abstractmethod


from typing import Tuple
from numpy.typing import NDArray


class TrajectoryGenerator(ABC):

    def __init__(self, name: str):
        self.name = name

    @abstractmethod
    def generate_trajectory(self, origin: Tuple[float, float]) -> NDArray:
        roscomp.logerr("This generator didn't implement a generator function.")
        return np.empty((0, 2))

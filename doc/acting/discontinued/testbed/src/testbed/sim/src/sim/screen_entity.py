import numpy as np
from numpy.typing import NDArray
from typing import List, Tuple


class ScreenEntity:

    def __init__(self, screen_factor: float, color: Tuple[int, int, int]):
        self._screen_factor = screen_factor
        self._color = color

    def _width_height_from_screen(self, screen) -> Tuple[int, int]:
        _, __, width, height = screen.get_rect()
        return (width, height)

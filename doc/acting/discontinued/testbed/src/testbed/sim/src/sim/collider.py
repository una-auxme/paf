import pygame
import numpy as np
from numpy.typing import NDArray

from typing import Tuple, List

from sim.screen_entity import ScreenEntity


class Collider:
    position: NDArray[np.float_] = np.array([0.0, 0.0])

    radius: float = 0.5  # m

    def __init__(self, x: float, y: float):
        self.set_position(x, y)

    def set_position(self, x: float, y: float) -> None:
        self.position = np.array([x, y])

    def get_position(self) -> NDArray[np.float_]:
        return self.position

    def to_other_coordinates(
        self, c_x: float, c_y: float, c_phi: float
    ) -> NDArray[np.float_]:
        translation = np.array([c_x, c_y])
        rotation_matrix = np.array(
            [[np.cos(-c_phi), -np.sin(-c_phi)], [np.sin(-c_phi), np.cos(-c_phi)]]
        )
        world_position = self.get_position()

        relative_position = world_position - translation
        other_position = rotation_matrix @ relative_position

        return other_position


class ScreenCollider(Collider, ScreenEntity):

    def __init__(
        self, x: float, y: float, screen_factor: float, color: Tuple[int, int, int]
    ):
        Collider.__init__(self, x, y)
        ScreenEntity.__init__(self, screen_factor, color)

    def set_position_screen(self, x: int, y: int) -> None:
        """
        X-Y in pygame left hand coordinate system need
        to be converted to right hand system
        """
        return super().set_position(y / self._screen_factor, x / self._screen_factor)

    def draw(self, screen):
        pygame.draw.circle(
            screen, self._color, self._position_to_screen(), self._radius_to_screen()
        )

    def _width_height_from_screen(self, screen) -> Tuple[int, int]:
        _, __, width, height = screen.get_rect()
        return (width, height)

    def _position_to_screen(self) -> NDArray[np.float_]:
        # Especially important right handed data to left handed draw
        return (self.position * self._screen_factor).astype(np.int_)[::-1]

    def _radius_to_screen(self) -> int:
        return int(self.radius * self._screen_factor)

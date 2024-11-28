from abc import ABC, abstractmethod
from dataclasses import dataclass


@dataclass
class Shape2D(ABC):
    @abstractmethod
    def check_collision(self, other) -> bool:
        raise NotImplementedError


@dataclass
class Rectangle(Shape2D):
    length: float
    width: float

    def check_collision(self, other) -> bool:
        return super().check_collision(other)


@dataclass
class Circle(Shape2D):
    radius: float

    def check_collision(self, other) -> bool:
        return super().check_collision(other)

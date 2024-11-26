from typing import List, Optional
from enum import Enum
from dataclasses import dataclass

from uuid import UUID, uuid4
from rospy import Time, Duration

from . import ROSMessageConvertible
from transform import Vector2, Transform2D
from shapes import Shape2D


@dataclass
class Motion2D(ROSMessageConvertible):
    linear_motion: Vector2
    angular_velocity: float


@dataclass
class Flags(ROSMessageConvertible):
    is_collider: bool
    is_tracked: bool
    is_stopmark: bool
    is_lanemark: bool
    is_ignored: bool


@dataclass
class TrackingInfo(ROSMessageConvertible):
    visibility_time: Duration
    invisibility_time: Duration
    visibility_frame_count: int
    invisibility_frame_count: int
    moving_time: Duration
    standing_time: Duration
    moving_time_sum: Duration
    standing_time_sum: Duration
    min_linear_speed: float
    max_linear_speed: float


@dataclass
class Entity(ROSMessageConvertible):
    uuid: UUID = uuid4()
    timestamp: Time
    confidence: float
    priority: float
    sensor_id: List[str] = []
    flags: Flags
    shape: Shape2D
    transform: Transform2D
    motion: Optional[Motion2D] = None
    tracking_info: Optional[TrackingInfo] = None


class Car(Entity):
    class BrakeLightState(Enum):
        OFF = 0
        ON = 1

    class IndicatorState(Enum):
        OFF = 0
        LEFT = 1
        RIGHT = 2

    def __init__(
        self,
        brake_light: "Car.BrakeLightState",
        indicator: "Car.IndicatorState",
        **kwargs
    ):
        super().__init__(**kwargs)
        self.brake_light = brake_light
        self.indicator = indicator


class Lanemarking(Entity):
    class Style(Enum):
        SOLID = 0
        DASHED = 1

    def __init__(self, style: "Lanemarking.Style", **kwargs):
        super().__init__(**kwargs)
        self.style = style


class TrafficLight(Entity):
    class State(Enum):
        RED = 0
        GREEN = 1
        YELLOW = 2

    def __init__(self, state: "TrafficLight.State", **kwargs):
        super().__init__(**kwargs)
        self.state = state


class Pedestrian(Entity):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


class Map(ROSMessageConvertible):
    timestamp: Time
    entities: List[Entity]

    def __init__(self, timestamp: Time, entities: List[Entity] = []):
        self.timestamp = timestamp
        self.entities = entities

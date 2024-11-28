from typing import List, Optional
from enum import Enum
from dataclasses import dataclass

from uuid import UUID, uuid4
from genpy.rostime import Time, Duration
import rospy

from transform import Vector2, Transform2D
from shapes import Shape2D

from mapping import msg


@dataclass
class Motion2D:
    linear_motion: Vector2
    angular_velocity: float


@dataclass
class Flags:
    is_collider: bool = False
    is_stopmark: bool = False
    is_lanemark: bool = False
    is_ignored: bool = False


@dataclass
class FlagFilter:
    has_motion: Optional[bool] = None
    is_collider: Optional[bool] = None
    is_tracked: Optional[bool] = None
    is_stopmark: Optional[bool] = None
    is_lanemark: Optional[bool] = None
    is_ignored: Optional[bool] = None


@dataclass
class TrackingInfo:
    visibility_time: Duration = Duration()
    invisibility_time: Duration = Duration()
    visibility_frame_count: int = 0
    invisibility_frame_count: int = 0
    moving_time: Duration = Duration()
    standing_time: Duration = Duration()
    moving_time_sum: Duration = Duration()
    standing_time_sum: Duration = Duration()
    min_linear_speed: float = 0.0
    max_linear_speed: float = 0.0

    @staticmethod
    def from_ros_msg(m: msg.TrackingInfo) -> "TrackingInfo":
        return TrackingInfo(
            visibility_time=m.visibility_time,
            invisibility_time=m.invisibility_time,
            visibility_frame_count=m.visibility_frame_count,
            invisibility_frame_count=m.invisibility_frame_count,
            moving_time=m.moving_time,
            standing_time=m.standing_time,
            moving_time_sum=m.moving_time_sum,
            standing_time_sum=m.standing_time_sum,
            min_linear_speed=m.min_linear_speed,
            max_linear_speed=m.max_linear_speed,
        )

    def to_ros_msg(self) -> msg.TrackingInfo:
        return msg.TrackingInfo(
            visibility_time=self.visibility_time,
            invisibility_time=self.invisibility_time,
            visibility_frame_count=self.visibility_frame_count,
            invisibility_frame_count=self.invisibility_frame_count,
            moving_time=self.moving_time,
            standing_time=self.standing_time,
            moving_time_sum=self.moving_time_sum,
            standing_time_sum=self.standing_time_sum,
            min_linear_speed=self.min_linear_speed,
            max_linear_speed=self.max_linear_speed,
        )


@dataclass
class Entity:
    timestamp: Time
    confidence: float
    priority: float
    flags: Flags
    shape: Shape2D
    transform: Transform2D
    uuid: UUID = uuid4()
    sensor_id: List[str] = []
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


class Map:
    timestamp: Time
    entities: List[Entity]

    def __init__(self, timestamp: Time, entities: List[Entity] = []):
        self.timestamp = timestamp
        self.entities = entities

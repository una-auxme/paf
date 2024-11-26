#!/usr/bin/env python


from typing import List, Optional

from enum import Enum


from abc import ABC, abstractmethod

from dataclasses import dataclass


@dataclass
class Vector2D:
    x: float
    y: float


@dataclass
class Object2D:
    translation: Vector2D
    rotation: float
    linear_motion: Vector2D
    angular_velocity: float
    motion_set: bool


@dataclass
class ShapedObject2D(Object2D, ABC):
    @abstractmethod
    def checkCollision(self, other):
        pass  # Placeholder for collision


@dataclass
class Rectangle(ShapedObject2D):
    length: float
    width: float

    def checkCollision(self, other):
        return super().checkCollision(other)


@dataclass
class Circle(ShapedObject2D):
    radius: float

    def checkCollision(self, other):
        return super().checkCollision(other)


@dataclass
class Filterable:
    is_collider: bool
    is_tracked: bool
    is_stopmark: bool
    is_lanemark: bool
    is_ignored: bool


@dataclass
class TrackingInfo:
    visibility_time: float
    invisibility_time: float
    visibility_frame_count: int
    invisibility_frame_count: int
    moving_time: float
    standing_time: float
    moving_time_sum: float
    standing_time_sum: float
    min_linear_speed: float
    max_linear_speed: float


@dataclass
class EntityInfo:
    map_id: int
    timestamp: float
    confidence: float
    priority: float
    sensor_id: List[str]
    has_tracking_info: bool
    tracking_info: Optional[TrackingInfo] = None


class Entity(EntityInfo, Filterable):
    def __init__(
        self,
        entity_info: EntityInfo,
        filterable: Filterable,
        shaped_object: ShapedObject2D,
    ):
        EntityInfo.__init__(
            self,
            entity_info.map_id,
            entity_info.timestamp,
            entity_info.confidence,
            entity_info.priority,
            entity_info.sensor_id,
            entity_info.has_tracking_info,
            entity_info.tracking_info,
        )
        Filterable.__init__(
            self,
            filterable.is_collider,
            filterable.is_tracked,
            filterable.is_stopmark,
            filterable.is_lanemark,
            filterable.is_ignored,
        )

        self.shape = shaped_object


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
        entity_info: EntityInfo,
        shaped_object: ShapedObject2D,
        filterable: Filterable,
        brake_light: "Car.BrakeLightState",
        indicator: "Car.IndicatorState",
    ):
        super().__init__(entity_info, filterable, shaped_object)
        self.brake_light = brake_light
        self.indicator = indicator


class Lanemarking(Entity):
    class Style(Enum):
        SOLID = 0
        DASHED = 1

    def __init__(
        self,
        entity_info: EntityInfo,
        filterable: Filterable,
        shaped_object: ShapedObject2D,
        style: "Lanemarking.Style",
    ):
        super().__init__(entity_info, filterable, shaped_object)

        self.style = style


class TrafficLight(Entity):
    class State(Enum):
        RED = 0
        GREEN = 1
        YELLOW = 2

    def __init__(
        self,
        entity_info: EntityInfo,
        filterable: Filterable,
        shaped_object: ShapedObject2D,
        state: "TrafficLight.State",
    ):
        super().__init__(entity_info, filterable, shaped_object)
        self.state = state


class Pedestrian(Entity):
    def __init__(
        self,
        entity_info: EntityInfo,
        filterable: Filterable,
        shaped_object: ShapedObject2D,
    ):
        super().__init__(entity_info, filterable, shaped_object)


class Map:
    def __init__(self, timestamp: float, entities: List[Entity]):
        self.timestamp = timestamp
        self.entities = entities

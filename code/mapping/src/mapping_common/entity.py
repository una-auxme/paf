from typing import List, Optional, Dict
from enum import Enum
from dataclasses import dataclass, field

from uuid import UUID, uuid4
from genpy.rostime import Time, Duration
from std_msgs.msg import Header
import uuid_msgs.msg as uuid_msgs
from visualization_msgs.msg import Marker
import rospy
from tf.transformations import quaternion_from_euler

from .transform import Vector2, Transform2D
from .shape import Shape2D

from mapping import msg


@dataclass
class Motion2D:
    linear_motion: Vector2 = field(default_factory=lambda: Vector2.zero())
    angular_velocity: float = 0.0

    @staticmethod
    def from_ros_msg(m: msg.Motion2D) -> "Motion2D":
        motion = Vector2.from_ros_msg(m.linear_motion)
        return Motion2D(linear_motion=motion, angular_velocity=m.angular_velocity)

    def to_ros_msg(self) -> msg.Motion2D:
        motion = self.linear_motion.to_ros_msg()
        return msg.Motion2D(
            linear_motion=motion, angular_velocity=self.angular_velocity
        )


@dataclass
class Flags:
    is_collider: bool = False
    is_stopmark: bool = False
    is_lanemark: bool = False
    is_ignored: bool = False

    @staticmethod
    def from_ros_msg(m: msg.Flags) -> "Flags":
        return Flags(
            is_collider=m.is_collider,
            is_stopmark=m.is_stopmark,
            is_lanemark=m.is_lanemark,
            is_ignored=m.is_ignored,
        )

    def to_ros_msg(self) -> msg.Flags:
        return msg.Flags(
            is_collider=self.is_collider,
            is_stopmark=self.is_stopmark,
            is_lanemark=self.is_lanemark,
            is_ignored=self.is_ignored,
        )


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
    visibility_time: Duration = field(default_factory=Duration)
    invisibility_time: Duration = field(default_factory=Duration)
    visibility_frame_count: int = 0
    invisibility_frame_count: int = 0
    moving_time: Duration = field(default_factory=Duration)
    standing_time: Duration = field(default_factory=Duration)
    moving_time_sum: Duration = field(default_factory=Duration)
    standing_time_sum: Duration = field(default_factory=Duration)
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
    confidence: float
    priority: float
    shape: Shape2D
    transform: Transform2D
    timestamp: Time = field(default_factory=Time)
    flags: Flags = field(default_factory=Flags)
    uuid: UUID = uuid4()
    sensor_id: List[str] = field(default_factory=list)
    motion: Optional[Motion2D] = None
    tracking_info: Optional[TrackingInfo] = None

    def matches_filter(self, f: FlagFilter) -> bool:
        raise NotImplementedError

    @staticmethod
    def from_ros_msg(m: msg.Entity) -> "Entity":
        entity_type = None
        msg_type_lower = m.type_name.lower()
        if msg_type_lower in _entity_supported_classes_dict:
            entity_type = _entity_supported_classes_dict[msg_type_lower]
        if entity_type is None:
            rospy.logerr(
                f"""Received entity type '{m.type_name}' is not supported.
Base class 'Entity' will be used instead.
The type must be one of {_entity_supported_classes_dict.keys()}"""
            )
            entity_type = Entity

        kwargs = entity_type._extract_kwargs(m)
        return entity_type(**kwargs)

    @staticmethod
    def _extract_kwargs(m: msg.Entity) -> Dict:
        motion = (
            None if m.flags.has_motion is False else Motion2D.from_ros_msg(m.motion)
        )
        tracking_info = (
            None
            if m.flags.is_tracked is False
            else TrackingInfo.from_ros_msg(m.tracking_info)
        )
        return {
            "confidence": m.confidence,
            "priority": m.priority,
            "shape": Shape2D.from_ros_msg(m.shape),
            "transform": Transform2D.from_ros_msg(m.transform),
            "timestamp": m.header.stamp,
            "flags": Flags.from_ros_msg(m.flags),
            "uuid": UUID(bytes=m.uuid.uuid),
            "sensor_id": m.sensor_id,
            "motion": motion,
            "tracking_info": tracking_info,
        }

    def to_ros_msg(self) -> msg.Entity:
        type_name = type(self).__name__
        flags = self.flags.to_ros_msg()
        flags.has_motion = self.motion is not None
        flags.is_tracked = self.tracking_info is not None
        motion = self.motion.to_ros_msg() if self.motion is not None else msg.Motion2D()
        tracking_info = (
            self.tracking_info.to_ros_msg()
            if self.tracking_info is not None
            else msg.TrackingInfo()
        )
        return msg.Entity(
            confidence=self.confidence,
            priority=self.priority,
            shape=self.shape.to_ros_msg(),
            transform=self.transform.to_ros_msg(),
            header=Header(stamp=self.timestamp),
            flags=flags,
            uuid=uuid_msgs.UniqueID(uuid=self.uuid.bytes),
            sensor_id=self.sensor_id,
            motion=motion,
            tracking_info=tracking_info,
            type_name=type_name,
        )

    def to_marker(self) -> Marker:
        m = self.shape.to_marker()

        m.color.a = 0.5
        m.color.r = 128
        m.color.g = 128
        m.color.b = 128

        transl = self.transform.translation()

        m.pose.position.x = transl.x()
        m.pose.position.y = transl.y()
        m.pose.position.z = m.scale.z / 2.0
        (
            m.pose.orientation.x,
            m.pose.orientation.y,
            m.pose.orientation.z,
            m.pose.orientation.w,
        ) = quaternion_from_euler(0, 0, 0)


@dataclass(init=False)
class Car(Entity):
    brake_light: "Car.BrakeLightState"
    indicator: "Car.IndicatorState"

    class BrakeLightState(Enum):
        OFF = 0
        ON = 1

    class IndicatorState(Enum):
        OFF = 0
        LEFT = 1
        RIGHT = 2

    def __init__(
        self,
        brake_light: "Car.BrakeLightState" = BrakeLightState.OFF,
        indicator: "Car.IndicatorState" = IndicatorState.OFF,
        **kwargs,
    ):
        super().__init__(**kwargs)
        self.brake_light = brake_light
        self.indicator = indicator

    @staticmethod
    def _extract_kwargs(m: msg.Entity) -> Dict:
        kwargs = super(Car, Car)._extract_kwargs(m)
        kwargs["brake_light"] = Car.BrakeLightState(m.type_car.brake_light)
        kwargs["indicator"] = Car.IndicatorState(m.type_car.indicator)
        return kwargs

    def to_ros_msg(self, base_msg: Optional[msg.Entity] = None) -> msg.Entity:
        m = super().to_ros_msg()
        m.type_car = msg.TypeCar(
            brake_light=self.brake_light.value, indicator=self.indicator.value
        )
        return m


@dataclass(init=False)
class Lanemarking(Entity):
    style: "Lanemarking.Style"

    class Style(Enum):
        SOLID = 0
        DASHED = 1

    def __init__(self, style: "Lanemarking.Style", **kwargs):
        super().__init__(**kwargs)
        self.style = style

    @staticmethod
    def _extract_kwargs(m: msg.Entity) -> Dict:
        kwargs = super(TrafficLight, TrafficLight)._extract_kwargs(m)
        kwargs["style"] = Lanemarking.Style(m.type_lanemarking.style)
        return kwargs

    def to_ros_msg(self, base_msg: Optional[msg.Entity] = None) -> msg.Entity:
        m = super().to_ros_msg()
        m.type_lanemarking = msg.TypeLanemarking(style=self.style.value)
        return m


@dataclass(init=False)
class TrafficLight(Entity):
    state: "TrafficLight.State"

    class State(Enum):
        RED = 0
        GREEN = 1
        YELLOW = 2

    def __init__(self, state: "TrafficLight.State", **kwargs):
        super().__init__(**kwargs)
        self.state = state

    @staticmethod
    def _extract_kwargs(m: msg.Entity) -> Dict:
        kwargs = super(TrafficLight, TrafficLight)._extract_kwargs(m)
        kwargs["state"] = TrafficLight.State(m.type_traffic_light.state)
        return kwargs

    def to_ros_msg(self, base_msg: Optional[msg.Entity] = None) -> msg.Entity:
        m = super().to_ros_msg()
        m.type_traffic_light = msg.TypeTrafficLight(state=self.state.value)
        return m


@dataclass(init=False)
class Pedestrian(Entity):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)


_entity_supported_classes = [Entity, Car, Lanemarking, TrafficLight, Pedestrian]
_entity_supported_classes_dict = {}
for t in _entity_supported_classes:
    t_name = t.__name__.lower()
    _entity_supported_classes_dict[t_name] = t

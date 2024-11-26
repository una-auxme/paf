#!/usr/bin/env python


import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
from visualization_msgs.msg import Marker, MarkerArray


from rospy import Publisher
from rospy import Duration

from entity import (
    Rectangle,
    Circle,
    Vector2D,
    Filterable,
    EntityInfo,
    Car,
    Pedestrian,
    Entity,
    Map,
)

from marker_entities import entity_to_marker

c_shaped_object = Rectangle(
    translation=Vector2D(10, 2),
    rotation=0.0,
    linear_motion=Vector2D(0, 0),
    angular_velocity=0.2,
    motion_set=True,
    length=2.0,
    width=4.0,
)

c_filterable = Filterable(
    is_collider=True,
    is_tracked=True,
    is_stopmark=True,
    is_lanemark=False,
    is_ignored=False,
)
c_entity_info = EntityInfo(
    map_id=1,
    timestamp=1.0,
    confidence=0.2,
    priority=0.3,
    sensor_id=[0],
    has_tracking_info=False,
)

car = Car(
    entity_info=c_entity_info,
    filterable=c_filterable,
    shaped_object=c_shaped_object,
    brake_light=Car.BrakeLightState.ON,
    indicator=Car.IndicatorState.LEFT,
)

p_shaped_object = Circle(
    translation=Vector2D(7, -3),
    rotation=0.0,
    linear_motion=Vector2D(0, 0),
    angular_velocity=0.0,
    motion_set=False,
    radius=0.5,
)

p_filterable = Filterable(
    is_collider=False,
    is_tracked=False,
    is_stopmark=False,
    is_lanemark=False,
    is_ignored=False,
)

p_entity_info = EntityInfo(
    map_id=2,
    timestamp=1.0,
    confidence=0.2,
    priority=0.2,
    sensor_id=[1],
    has_tracking_info=False,
)

pedestrian = Pedestrian(p_entity_info, p_filterable, p_shaped_object)

map = Map(1.0, [car, pedestrian])


class Visualization(CompatibleNode):
    """The visualization for the intermediate layer.

    This Node will publish the marker array composed of the different entities.
    """

    def __init__(self, args):
        super().__init__("intermediate_layer_visualization")

        self.marker_publisher: Publisher = self.new_publisher(
            MarkerArray, "/marker_array", qos_profile=10
        )

        self.filter_is_collider = False
        self.filter_is_tracked = False
        self.filter_is_stopmark = False
        self.filter_is_lanemark = False
        self.filter_is_ignored = False

        self.new_service(SetBool, "vis/set_is_collider", self.set_is_collider_filter)
        self.new_service(SetBool, "vis/set_is_tracked", self.set_is_tracked_filter)
        self.new_service(SetBool, "vis/set_is_stopmark", self.set_is_stopmark_filter)
        self.new_service(SetBool, "vis/set_is_lanemark", self.set_is_lanemark_filter)
        self.new_service(SetBool, "vis/set_is_ignored", self.set_is_ignored_filter)

        self.rate = 1.0 / 20.0
        self.new_timer(self.rate, self.loop)

    def loop(self, timer_event=None):
        """This loop will run with self.rate hz.
        Its purpose is to process data and publish as marker array.
        """
        marker_array = MarkerArray()

        for entity in map.entities:
            marker_array.markers.append(self.create_marker_from_entity(entity))

        self.marker_publisher.publish(marker_array)

    def create_marker_from_entity(self, entity: Entity):
        marker = Marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = roscomp.ros_timestamp(self.get_time(), from_sec=True)
        marker.ns = "m"
        marker.id = entity.map_id
        marker.lifetime = Duration(1)

        if self.filter_is_collider:
            if entity.is_collider:
                marker.action = Marker.ADD
            else:
                marker.action = Marker.DELETE

        entity_to_marker(entity, marker)
        return marker

    def set_is_collider_filter(self, request: SetBoolRequest):
        self.filter_is_collider = request.data
        self.loginfo(f"Called collider {request.data}")

        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_tracked_filter(self, request: SetBoolRequest):
        self.filter_is_tracked = request.data
        self.loginfo(f"Called tracked {request.data}")

        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_stopmark_filter(self, request: SetBoolRequest):
        self.filter_is_stopmark = request.data
        self.loginfo(f"Called stopmark {request.data}")
        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_lanemark_filter(self, request: SetBoolRequest):
        self.filter_is_lanemark = request.data

        self.loginfo(f"Called lanemark {request.data}")
        response = SetBoolResponse()
        response.success = True
        return response

    def set_is_ignored_filter(self, request: SetBoolRequest):
        self.filter_is_ignored = request.data

        self.loginfo(f"Called ignored {request.data}")
        response = SetBoolResponse()
        response.success = True
        return response


def main(args=None):
    """Main function to start the node.

    Args:
        args (_type_, optional): Runtime args, do not get processed. Defaults to None.
    """

    roscomp.init("intermediate_layer_visualization")

    try:
        node = Visualization(args)
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

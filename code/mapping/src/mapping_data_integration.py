#!/usr/bin/env python


from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import ros_numpy
import rospy
from visualization_msgs.msg import Marker
import numpy as np
from typing import List, Optional, Dict
from copy import deepcopy

import mapping_common.map
import mapping_common.hero
from mapping_common.entity import Entity, Flags, Car, Motion2D, Pedestrian, StopMark
from mapping_common.transform import Transform2D, Vector2, Point2
from mapping_common.shape import Circle, Polygon, Rectangle
from mapping_common.map import Map
from mapping_common.filter import (
    MapFilter,
    GrowthMergingFilter,
    LaneIndexFilter,
    GrowPedestriansFilter,
)

from mapping.msg import Map as MapMsg, ClusteredPointsArray
from mapping.srv import UpdateStopMarks, UpdateStopMarksRequest, UpdateStopMarksResponse
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from carla_msgs.msg import CarlaSpeedometer
from shapely.geometry import MultiPoint
import shapely

from mapping.cfg import MappingIntegrationConfig
from dynamic_reconfigure.server import Server


class MappingDataIntegrationNode(CompatibleNode):
    """Creates the initial map data frame based on all kinds of sensor data

    It applies several filters to the map and
    then sends it off to other consumers (planning, acting)

    This node sends the maps off at a fixed rate.
    (-> It buffers incoming sensor data slightly)
    """

    lidar_data: Optional[PointCloud2] = None
    hero_speed: Optional[CarlaSpeedometer] = None
    lidar_clustered_points_data: Optional[ClusteredPointsArray] = None
    radar_clustered_points_data: Optional[ClusteredPointsArray] = None
    vision_clustered_points_data: Optional[ClusteredPointsArray] = None

    stop_marks: Dict[str, List[StopMark]]
    """StopMarks from the UpdateStopMarks service

    **Important:** the transform of these entities uses global coordinates
    """
    current_pos: Optional[Point2] = None
    current_heading: Optional[float] = None

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # For the stop marks:
        self.stop_marks = {}

        self.current_pos_sub = self.new_subscription(
            PoseStamped,
            "/paf/hero/current_pos",
            self.current_pos_callback,
            qos_profile=1,
        )
        self.head_sub = self.new_subscription(
            Float32,
            "/paf/hero/current_heading",
            self.heading_callback,
            qos_profile=1,
        )
        self.update_stop_marks_service = rospy.Service(
            "/paf/hero/mapping/update_stop_marks",
            UpdateStopMarks,
            self.update_stopmarks_callback,
        )

        # Sensor subscriptions:

        self.lanemarkings = None

        self.new_subscription(
            topic=self.get_param("~lidar_topic", "/carla/hero/LIDAR"),
            msg_type=PointCloud2,
            callback=self.lidar_callback,
            qos_profile=1,
        )

        self.new_subscription(
            topic=self.get_param(
                "~lanemarkings_init_topic", "/paf/hero/mapping/init_lanemarkings"
            ),
            msg_type=MapMsg,
            callback=self.lanemarkings_callback,
            qos_profile=1,
        )
        self.new_subscription(
            topic=self.get_param("~hero_speed_topic", "/carla/hero/Speed"),
            msg_type=CarlaSpeedometer,
            callback=self.hero_speed_callback,
            qos_profile=1,
        )
        self.new_subscription(
            topic=self.get_param(
                "~clustered_points_lidar_topic", "/paf/hero/Lidar/clustered_points"
            ),
            msg_type=ClusteredPointsArray,
            callback=self.lidar_clustered_points_callback,
            qos_profile=1,
        )
        self.new_subscription(
            topic=self.get_param(
                "~clustered_points_vision_topic", "/paf/hero/visualization_pointcloud"
            ),
            msg_type=ClusteredPointsArray,
            callback=self.vision_clustered_points_callback,
            qos_profile=1,
        )
        self.new_subscription(
            topic=self.get_param(
                "~clustered_points_radar_topic", "/paf/hero/Radar/clustered_points"
            ),
            msg_type=ClusteredPointsArray,
            callback=self.radar_clustered_points_callback,
            qos_profile=1,
        )

        # Publishers:

        self.map_publisher = self.new_publisher(
            msg_type=MapMsg,
            topic=self.get_param("~map_init_topic", "/paf/hero/mapping/init_data"),
            qos_profile=1,
        )

        self.cluster_points_publisher = self.new_publisher(
            msg_type=PointCloud2,
            topic=self.get_param(
                "~cluster_points_topic", "/paf/hero/mapping/clusterpoints"
            ),
            qos_profile=1,
        )

        Server(MappingIntegrationConfig, self.dynamic_reconfigure_callback)

        self.rate = self.get_param("~map_publish_rate", 20)
        self.new_timer(1.0 / self.rate, self.publish_new_map_handler)

    def dynamic_reconfigure_callback(self, config: "MappingIntegrationConfig", level):
        """
        All currently used reconfigure options are querried dynamically.
        If you want to directly react on the change uncomment the following lines.
        """
        # config["enable_merge_filter"]
        # config["merge_growth_distance"]
        # config["min_merging_overlap_percent"]
        # config["min_merging_overlap_area"]
        return config

    def update_stopmarks_callback(
        self, req: UpdateStopMarksRequest
    ) -> UpdateStopMarksResponse:
        if req.delete_all_others:
            self.stop_marks = {}

        entities = []
        for e in req.marks:
            entity = Entity.from_ros_msg(e)
            if not isinstance(entity, StopMark):
                rospy.logwarn_throttle(
                    0.5,
                    "Entity received from UpdateStopMarks service is not a StopMark."
                    " ignoring...",
                )
                continue
            entities.append(entity)
        self.stop_marks[req.id] = entities

        return UpdateStopMarksResponse(success=True)

    def heading_callback(self, data: Float32):
        self.current_heading = data.data

    def current_pos_callback(self, data: PoseStamped):
        self.current_pos = Point2.from_ros_msg(data.pose.position)

    def hero_speed_callback(self, data: CarlaSpeedometer):
        self.hero_speed = data

    def lidar_clustered_points_callback(self, data: ClusteredPointsArray):
        self.lidar_clustered_points_data = data

    def radar_clustered_points_callback(self, data: ClusteredPointsArray):
        self.radar_clustered_points_data = data

    def vision_clustered_points_callback(self, data: ClusteredPointsArray):
        self.vision_clustered_points_data = data

    def lidar_callback(self, data: PointCloud2):
        self.lidar_data = data

    def entities_from_lidar_marker(self) -> List[Entity]:
        data = self.lidar_marker_data
        if data is None or not hasattr(data, "markers") or data.markers is None:
            rospy.logwarn("No valid marker data received.")
            return []

        lidar_entities = []
        for marker in data.markers:
            if marker.type != Marker.CUBE:
                rospy.logwarn(f"Skipping non-CUBE marker with ID: {marker.id}")
                continue
            # Extract position (center of the cube)
            x_center = marker.pose.position.x
            y_center = marker.pose.position.y

            # Extract dimensions (scale gives the size of the cube)
            width = marker.scale.x
            length = marker.scale.y

            # Create a shape and transform using the cube's data
            shape = Rectangle(width, length)  # 2D rectangle for lidar data
            v = Vector2.new(x_center, y_center)  # 2D position in x-y plane
            transform = Transform2D.new_translation(v)

            # Add entity to the list
            flags = Flags(is_collider=True)
            e = Entity(
                confidence=1,
                priority=0.25,
                shape=shape,
                transform=transform,
                timestamp=marker.header.stamp,
                flags=flags,
            )
            lidar_entities.append(e)

        return lidar_entities

    def entities_from_radar_marker(self) -> List[Entity]:
        data = self.radar_marker_data
        if data is None or not hasattr(data, "markers") or data.markers is None:
            # Handle cases where data or markers are invalid
            rospy.logwarn("No valid marker data received.")
            return []

        radar_entities = []
        for marker in data.markers:
            if marker.type != Marker.CUBE:
                rospy.logwarn(f"Skipping non-CUBE marker with ID: {marker.id}")
                continue
            # Extract position (center of the cube)
            x_center = marker.pose.position.x
            y_center = marker.pose.position.y

            # Extract dimensions (scale gives the size of the cube)
            width = marker.scale.x
            length = marker.scale.y

            # Create a shape and transform using the cube's data
            shape = Rectangle(width, length)  # 2D rectangle for lidar data
            v = Vector2.new(x_center, y_center)  # 2D position in x-y plane
            transform = Transform2D.new_translation(v)

            # Add entity to the list
            flags = Flags(is_collider=True)
            e = Entity(
                confidence=1,
                priority=0.25,
                shape=shape,
                transform=transform,
                timestamp=marker.header.stamp,
                flags=flags,
            )
            radar_entities.append(e)

        return radar_entities

    def lanemarkings_callback(self, data: MapMsg):
        map = Map.from_ros_msg(data)
        self.lanemarkings = map.entities_without_hero()

    def entities_from_lidar(self) -> List[Entity]:
        if self.lidar_data is None:
            return []

        data = self.lidar_data
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        coordinates = coordinates.view(
            (coordinates.dtype[0], len(coordinates.dtype.names))
        )
        shape = Circle(self.get_param("~lidar_shape_radius", 0.15))
        z_min = self.get_param("~lidar_z_min", -1.5)
        z_max = self.get_param("~lidar_z_max", 1.0)
        priority = self.get_param("~lidar_priority", 0.25)

        # Ignore street level lidar points and stuff above
        filtered_coordinates = coordinates[
            np.bitwise_and(coordinates[:, 2] >= z_min, coordinates[:, 2] <= z_max)
        ]
        # Get rid of points because performance
        coordinate_count = filtered_coordinates.shape[0]
        sampled_coordinates = filtered_coordinates[
            np.random.choice(
                coordinate_count,
                int(
                    coordinate_count
                    * (1.0 - self.get_param("~lidar_discard_probability", 0.9))
                ),
                replace=False,
            ),
            :,
        ]
        lidar_entities = []
        for x, y, z, intensity in sampled_coordinates:
            v = Vector2.new(x, y)
            transform = Transform2D.new_translation(v)
            flags = Flags(is_collider=True)
            e = Entity(
                confidence=0.5 * intensity,
                priority=priority,
                shape=shape,
                transform=transform,
                timestamp=data.header.stamp,
                flags=flags,
            )
            lidar_entities.append(e)

        return lidar_entities

    def create_entities_from_clusters(self, sensortype="") -> List[Entity]:
        data = None
        if sensortype == "radar":
            data = self.radar_clustered_points_data
        elif sensortype == "lidar":
            data = self.lidar_clustered_points_data
        elif sensortype == "vision":
            data = self.vision_clustered_points_data
        else:
            raise ValueError(f"Unknown sensortype: {sensortype}")

        if data is None:
            return []

        clusterpointsarray = np.array(data.clusterPointsArray).reshape(-1, 3)

        indexarray = np.array(data.indexArray)

        motion_array_converted = (
            np.array([Motion2D.from_ros_msg(m) for m in data.motionArray])
            if data.motionArray
            else None
        )

        objectclassarray = np.array(data.object_class) if data.object_class else None

        unique_labels = np.unique(indexarray)

        entities = []
        for label in unique_labels:
            if label == -1:
                # -1 noise or invalid cluster
                rospy.logwarn("label -1")
                continue

            # Filter points for current cluster
            cluster_mask = indexarray == label
            cluster_points_xy = clusterpointsarray[cluster_mask, :2]

            # Check if enough points for polygon are available
            if cluster_points_xy.shape[0] < 3:
                if sensortype == "radar":
                    shape = Circle(self.get_param("~lidar_shape_radius", 0.15))
                    transform = Transform2D.new_translation(
                        Vector2.new(cluster_points_xy[0, 0], cluster_points_xy[0, 1])
                    )
                else:
                    continue
            else:
                if not np.array_equal(cluster_points_xy[0], cluster_points_xy[-1]):
                    # add startpoint to close polygon
                    cluster_points_xy = np.vstack(
                        [cluster_points_xy, cluster_points_xy[0]]
                    )

                cluster_polygon = MultiPoint(cluster_points_xy)
                cluster_polygon_hull = cluster_polygon.convex_hull
                if cluster_polygon_hull.is_empty or not cluster_polygon_hull.is_valid:
                    rospy.loginfo("Empty hull")
                    continue
                if not isinstance(cluster_polygon_hull, shapely.Polygon):
                    rospy.loginfo("Cluster is not polygon, continue")
                    continue

                shape = Polygon.from_shapely(
                    cluster_polygon_hull, make_centered=True  # type: ignore
                )
                transform = shape.offset
                shape.offset = Transform2D.identity()

            motion = None
            if motion_array_converted is not None:
                motion = motion_array_converted[cluster_mask][0]
                if self.hero_speed is not None:
                    motion_vector_hero = Vector2.forward() * self.hero_speed.speed
                    motion = Motion2D(
                        motion_vector_hero + motion.linear_motion, angular_velocity=0.0
                    )

            # Optional: Füge die Objektklasse hinzu
            object_class = None
            if objectclassarray is not None:
                object_class = objectclassarray[indexarray == label][0]

            flags = Flags(is_collider=True)
            if object_class == 4:
                entity = Pedestrian(
                    confidence=1,
                    priority=0.9,
                    shape=shape,
                    transform=transform,
                    timestamp=rospy.Time.now(),
                    flags=flags,
                    motion=motion,
                )
            elif object_class == 10:
                entity = Car(
                    confidence=1,
                    priority=0.75,
                    shape=shape,
                    transform=transform,
                    timestamp=rospy.Time.now(),
                    flags=flags,
                    motion=motion,
                )
            else:
                entity = Entity(
                    confidence=1,
                    priority=0.25,
                    shape=shape,
                    transform=transform,
                    timestamp=rospy.Time.now(),
                    flags=flags,
                    motion=motion,
                )
            entities.append(entity)

        return entities

    def create_hero_entity(self) -> Optional[Car]:
        if self.hero_speed is None:
            return None

        motion = Motion2D(Vector2.forward() * self.hero_speed.speed)
        timestamp = self.hero_speed.header.stamp
        hero = mapping_common.hero.create_hero_entity()
        hero.timestamp = timestamp
        hero.motion = motion
        return hero

    def publish_new_map_handler(self, timer_event=None):
        try:
            self.publish_new_map()
        except Exception as e:
            rospy.logfatal(f"Mapping data integration: {e}")

    def publish_new_map(self, timer_event=None):
        hero_car = self.create_hero_entity()
        if hero_car is None or self.current_pos is None or self.current_heading is None:
            return

        entities: List[Entity] = []
        entities.append(hero_car)

        if self.get_param("~enable_lidar_cluster"):
            if self.lidar_clustered_points_data is not None:
                entities.extend(self.create_entities_from_clusters(sensortype="lidar"))
            else:
                return

        if self.get_param("~enable_radar_cluster"):
            if self.radar_clustered_points_data is not None:
                entities.extend(self.create_entities_from_clusters(sensortype="radar"))
            else:
                return

        if self.get_param("~enable_vision_cluster"):
            if self.vision_clustered_points_data is not None:
                entities.extend(self.create_entities_from_clusters(sensortype="vision"))
            else:
                return

        if self.get_param("~enable_lane_marker"):
            if self.lanemarkings is not None:
                entities.extend(self.lanemarkings)
            else:
                return

        if self.get_param("~enable_raw_lidar_points"):
            if self.lidar_data is not None:
                entities.extend(self.entities_from_lidar())
            else:
                return

        if self.get_param("~enable_stop_marks"):
            hero_transform_inv = mapping_common.map.build_global_hero_transform(
                self.current_pos.x(),
                self.current_pos.y(),
                self.current_heading,
            ).inverse()
            marks = []
            for global_marks in self.stop_marks.values():
                for global_mark in global_marks:
                    local_mark = deepcopy(global_mark)
                    local_mark.transform = hero_transform_inv * local_mark.transform
                    marks.append(local_mark)

            entities.extend(marks)

        stamp = rospy.get_rostime()
        map = Map(timestamp=stamp, entities=entities)

        for filter in self.get_current_map_filters():
            map = filter.filter(map)
        msg = map.to_ros_msg()
        self.map_publisher.publish(msg)

    def get_current_map_filters(self) -> List[MapFilter]:
        map_filters: List[MapFilter] = []

        if self.get_param("~enable_merge_filter"):
            map_filters.append(
                GrowthMergingFilter(
                    growth_distance=self.get_param("~merge_growth_distance"),
                    min_merging_overlap_percent=self.get_param(
                        "~min_merging_overlap_percent"
                    ),
                    min_merging_overlap_area=self.get_param(
                        "~min_merging_overlap_area"
                    ),
                    simplify_tolerance=self.get_param("~polygon_simplify_tolerance"),
                )
            )
        if self.get_param("~enable_lane_index_filter"):
            map_filters.append(LaneIndexFilter())
        if self.get_param("~enable_pedestrian_grow_filter"):
            map_filters.append(GrowPedestriansFilter())

        return map_filters


if __name__ == "__main__":
    name = "mapping_data_integration"
    roscomp.init(name)
    node = MappingDataIntegrationNode(name)
    node.spin()

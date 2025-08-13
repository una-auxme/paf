from visualization_msgs.msg import Marker
import numpy as np
from typing import List, Optional, Dict
from copy import deepcopy

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.time import Time
import ros2_numpy

from paf_common.parameters import update_attributes
from paf_common.exceptions import emsg_with_trace
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

from mapping_interfaces.msg import Map as MapMsg, ClusteredPointsArray
from mapping_interfaces.srv import UpdateStopMarks
from rcl_interfaces.msg import (
    ParameterDescriptor,
    FloatingPointRange,
)
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import PointCloud2
from carla_msgs.msg import CarlaSpeedometer
from shapely.geometry import MultiPoint
import shapely


class MappingDataIntegrationNode(Node):
    """Creates the initial map data frame based on all kinds of sensor data.

    It applies several filters to the map and
    then sends it off to other consumers (planning, acting)

    This node sends the maps off at a fixed rate.
    (-> It buffers incoming sensor data slightly)

    #### Services

    This node provides the following services:

    - **UpdateStopMarks**:
      Allows a client to insert virtual StopMark Entities into the Map.

      Each list of StopMarks has a unique id. With this id, a client
      can update their specific list of marks.
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

    def __init__(self):
        super().__init__("mapping_data_integration")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        mapping_common.set_logger(self.get_logger())

        # Parameters

        self.map_publish_rate = (
            self.declare_parameter("map_publish_rate", 0.05)
            .get_parameter_value()
            .double_value
        )

        # Parameters: Enable entity sources

        self.enable_radar_cluster = (
            self.declare_parameter(
                "enable_radar_cluster",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable Radar Cluster integration",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.enable_lidar_cluster = (
            self.declare_parameter(
                "enable_lidar_cluster",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable Lidar Cluster integration",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.enable_vision_cluster = (
            self.declare_parameter(
                "enable_vision_cluster",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable Vision Node Cluster integration",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.enable_raw_lidar_points = (
            self.declare_parameter(
                "enable_raw_lidar_points",
                False,
                descriptor=ParameterDescriptor(
                    description="Enable raw lidar input",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.enable_lane_marker = (
            self.declare_parameter(
                "enable_lane_marker",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable Lane Mark integration",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.enable_stop_marks = (
            self.declare_parameter(
                "enable_stop_marks",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable stop marks from the UpdateStopMarks service",
                ),
            )
            .get_parameter_value()
            .bool_value
        )

        # Parameters: Filtering

        self.filter_enable_lane_index = (
            self.declare_parameter(
                "filter_enable_lane_index",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable or disable the lane index filter",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.filter_enable_pedestrian_grow = (
            self.declare_parameter(
                "filter_enable_pedestrian_grow",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable or disable the pedestrian grow filter",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.filter_enable_merge = (
            self.declare_parameter(
                "filter_enable_merge",
                True,
                descriptor=ParameterDescriptor(
                    description="Enable or disable the merging filter",
                ),
            )
            .get_parameter_value()
            .bool_value
        )
        self.filter_merge_growth_distance = (
            self.declare_parameter(
                "filter_merge_growth_distance",
                0.3,
                descriptor=ParameterDescriptor(
                    description="Amount shapes grow before merging in meters",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.01, to_value=5.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.filter_merge_min_overlap_percent = (
            self.declare_parameter(
                "filter_merge_min_overlap_percent",
                0.5,
                descriptor=ParameterDescriptor(
                    description="Min overlap of the grown shapes in percent",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.filter_merge_min_overlap_area = (
            self.declare_parameter(
                "filter_merge_min_overlap_area",
                0.5,
                descriptor=ParameterDescriptor(
                    description="Min overlap of the grown shapes in m2",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=5.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.polygon_simplify_tolerance = (
            self.declare_parameter(
                "polygon_simplify_tolerance",
                0.1,
                descriptor=ParameterDescriptor(
                    description="The polygon simplify tolerance",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.01, to_value=1.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )

        # Parameters: Lidar (Only relevant for the raw lider point input)

        self.lidar_z_min = (
            self.declare_parameter(
                "lidar_z_min",
                -1.5,
                descriptor=ParameterDescriptor(
                    description="Excludes lidar points below this height",
                    floating_point_range=[
                        FloatingPointRange(from_value=-10.0, to_value=2.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.lidar_z_max = (
            self.declare_parameter(
                "lidar_z_max",
                1.0,
                descriptor=ParameterDescriptor(
                    description="Exclude lidar points above this height",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.lidar_shape_radius = (
            self.declare_parameter(
                "lidar_shape_radius",
                0.15,
                descriptor=ParameterDescriptor(
                    description="The radius with which lidar points get added to map",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.lidar_priority = (
            self.declare_parameter(
                "lidar_priority",
                0.25,
                descriptor=ParameterDescriptor(
                    description="The priority lidar points have in the map",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.lidar_discard_probability = (
            self.declare_parameter(
                "lidar_discard_probability",
                0.9,
                descriptor=ParameterDescriptor(
                    description="Discard this many lidar points. "
                    "Important for performance",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=1.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )

        # For the stop marks:
        self.stop_marks = {}

        self.current_pos_sub = self.create_subscription(
            PoseStamped,
            "/paf/hero/current_pos",
            self.current_pos_callback,
            qos_profile=1,
        )
        self.head_sub = self.create_subscription(
            Float32,
            "/paf/hero/current_heading",
            self.heading_callback,
            qos_profile=1,
        )
        self.update_stop_marks_service = self.create_service(
            UpdateStopMarks,
            "/paf/hero/mapping/update_stop_marks",
            self.update_stopmarks_callback,
        )

        # Sensor subscriptions:
        self.lanemarkings = None

        self.create_subscription(
            topic="/carla/hero/LIDAR",
            msg_type=PointCloud2,
            callback=self.lidar_callback,
            qos_profile=1,
        )

        self.create_subscription(
            topic="/paf/hero/mapping/init_lanemarkings",
            msg_type=MapMsg,
            callback=self.lanemarkings_callback,
            qos_profile=1,
        )
        self.create_subscription(
            topic="/carla/hero/Speed",
            msg_type=CarlaSpeedometer,
            callback=self.hero_speed_callback,
            qos_profile=1,
        )
        self.create_subscription(
            topic="/paf/hero/Lidar/clustered_points",
            msg_type=ClusteredPointsArray,
            callback=self.lidar_clustered_points_callback,
            qos_profile=1,
        )
        self.create_subscription(
            topic="/paf/hero/visualization_pointcloud",
            msg_type=ClusteredPointsArray,
            callback=self.vision_clustered_points_callback,
            qos_profile=1,
        )
        self.create_subscription(
            topic="/paf/hero/Radar/clustered_points",
            msg_type=ClusteredPointsArray,
            callback=self.radar_clustered_points_callback,
            qos_profile=1,
        )

        # Publishers:

        self.map_publisher = self.create_publisher(
            msg_type=MapMsg,
            topic="/paf/hero/mapping/init_data",
            qos_profile=1,
        )

        self.cluster_points_publisher = self.create_publisher(
            msg_type=PointCloud2,
            topic="/paf/hero/mapping/clusterpoints",
            qos_profile=1,
        )

        self.create_timer(self.map_publish_rate, self.publish_new_map_handler)
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def update_stopmarks_callback(
        self, request: UpdateStopMarks.Request, response: UpdateStopMarks.Response
    ) -> UpdateStopMarks.Response:
        if request.delete_all_others:
            self.stop_marks = {}

        entities = []
        for e in request.marks:
            entity = Entity.from_ros_msg(e)
            if not isinstance(entity, StopMark):
                self.get_logger().warn(
                    "Entity received from UpdateStopMarks service is not a StopMark."
                    " ignoring...",
                    throttle_duration=0.5,
                )
                continue
            entities.append(entity)
        self.stop_marks[request.id] = entities

        response.success = True
        return response

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
            self.get_logger().warn("No valid marker data received.")
            return []

        lidar_entities = []
        for marker in data.markers:
            if marker.type != Marker.CUBE:
                self.get_logger().warn(f"Skipping non-CUBE marker with ID: {marker.id}")
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
            self.get_logger().warn("No valid marker data received.")
            return []

        radar_entities = []
        for marker in data.markers:
            if marker.type != Marker.CUBE:
                self.get_logger().warn(f"Skipping non-CUBE marker with ID: {marker.id}")
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
        coordinates = ros2_numpy.point_cloud2.pointcloud2_to_array(data)
        coordinates = coordinates.view(
            (coordinates.dtype[0], len(coordinates.dtype.names))
        )
        shape = Circle(self.lidar_shape_radius)
        z_min = self.lidar_z_min
        z_max = self.lidar_z_max
        priority = self.lidar_priority

        # Ignore street level lidar points and stuff above
        filtered_coordinates = coordinates[
            np.bitwise_and(coordinates[:, 2] >= z_min, coordinates[:, 2] <= z_max)
        ]
        # Get rid of points because performance
        coordinate_count = filtered_coordinates.shape[0]
        sampled_coordinates = filtered_coordinates[
            np.random.choice(
                coordinate_count,
                int(coordinate_count * (1.0 - self.lidar_discard_probability)),
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
                timestamp=Time.from_msg(data.header.stamp),
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

        clusterpointsarray = np.array(data.cluster_points_array).reshape(-1, 3)

        indexarray = np.array(data.index_array)

        motion_array_converted = (
            np.array([Motion2D.from_ros_msg(m) for m in data.motion_array])
            if data.motion_array
            else None
        )

        objectclassarray = np.array(data.object_class) if data.object_class else None

        unique_labels = np.unique(indexarray)

        entities = []
        for label in unique_labels:
            if label == -1:
                # -1 noise or invalid cluster
                self.get_logger().warn("label -1")
                continue

            # Filter points for current cluster
            cluster_mask = indexarray == label
            cluster_points_xy = clusterpointsarray[cluster_mask, :2]

            # Check if enough points for polygon are available
            if cluster_points_xy.shape[0] < 3:
                if sensortype == "radar":
                    shape = Circle(self.lidar_shape_radius)
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
                    self.get_logger().info("Empty hull", throttle_duration_sec=0.5)
                    continue
                if not isinstance(cluster_polygon_hull, shapely.Polygon):
                    self.get_logger().info(
                        "Cluster is not polygon, continue", throttle_duration_sec=0.5
                    )
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
            timestamp = self.get_clock().now()
            if object_class == 4:
                entity = Pedestrian(
                    confidence=1,
                    priority=0.9,
                    shape=shape,
                    transform=transform,
                    timestamp=timestamp,
                    flags=flags,
                    motion=motion,
                )
            elif object_class == 10:
                entity = Car(
                    confidence=1,
                    priority=0.75,
                    shape=shape,
                    transform=transform,
                    timestamp=timestamp,
                    flags=flags,
                    motion=motion,
                )
            else:
                entity = Entity(
                    confidence=1,
                    priority=0.25,
                    shape=shape,
                    transform=transform,
                    timestamp=timestamp,
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
        hero.timestamp = Time.from_msg(timestamp)
        hero.motion = motion
        return hero

    def publish_new_map_handler(self):
        try:
            self.publish_new_map()
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)

    def publish_new_map(self):
        """Publishes a new map with the currently available data.

        Args:
            timer_event (_type_, optional): Defaults to None.
        """
        hero_car = self.create_hero_entity()
        if hero_car is None or self.current_pos is None or self.current_heading is None:
            return

        entities: List[Entity] = []
        entities.append(hero_car)

        missing_data = []
        if self.enable_lidar_cluster:
            if self.lidar_clustered_points_data is not None:
                entities.extend(self.create_entities_from_clusters(sensortype="lidar"))
            else:
                missing_data.append("lidar_clustered_points")

        if self.enable_radar_cluster:
            if self.radar_clustered_points_data is not None:
                entities.extend(self.create_entities_from_clusters(sensortype="radar"))
            else:
                missing_data.append("radar_clustered_points")

        if self.enable_vision_cluster:
            if self.vision_clustered_points_data is not None:
                entities.extend(self.create_entities_from_clusters(sensortype="vision"))
            else:
                missing_data.append("vision_clustered_points")

        if self.enable_lane_marker:
            if self.lanemarkings is not None:
                entities.extend(self.lanemarkings)
            else:
                missing_data.append("lanemarkings")

        if self.enable_raw_lidar_points:
            if self.lidar_data is not None:
                entities.extend(self.entities_from_lidar())
            else:
                missing_data.append("raw_lidar_points")

        if missing_data:
            self.get_logger().warn(
                f"Missing data: {missing_data}. Unable to publish map.",
                throttle_duration_sec=2.0,
            )
            return

        if self.enable_stop_marks:
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

        stamp = self.get_clock().now()
        map = Map(timestamp=stamp, entities=entities)

        for filter in self.get_current_map_filters():
            map = filter.filter(map)
        msg = map.to_ros_msg()
        self.map_publisher.publish(msg)

    def get_current_map_filters(self) -> List[MapFilter]:
        """Creates an array of filters for the Map
        based on the parameters of this node.

        Returns:
            List[MapFilter]: Filters to apply to the Map
        """
        map_filters: List[MapFilter] = []

        if self.filter_enable_merge:
            map_filters.append(
                GrowthMergingFilter(
                    growth_distance=self.filter_merge_growth_distance,
                    min_merging_overlap_percent=self.filter_merge_min_overlap_percent,
                    min_merging_overlap_area=self.filter_merge_min_overlap_area,
                    simplify_tolerance=self.polygon_simplify_tolerance,
                )
            )
        if self.filter_enable_lane_index:
            map_filters.append(LaneIndexFilter())
        if self.filter_enable_pedestrian_grow:
            map_filters.append(GrowPedestriansFilter())

        return map_filters


def main(args=None):
    rclpy.init(args=args)

    try:
        node = MappingDataIntegrationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

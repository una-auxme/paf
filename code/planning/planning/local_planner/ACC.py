import math
from typing import Optional, List, Tuple

import shapely

import rclpy
from rclpy.node import Node
from rclpy.publisher import Publisher
from rclpy.subscription import Subscription
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange
from paf_common.parameters import update_attributes

from nav_msgs.msg import Path
from std_msgs.msg import Float32, String, Bool
from visualization_msgs.msg import Marker, MarkerArray

import mapping_common.markers
import mapping_common.shape
import mapping_common.map
import mapping_common.mask
import mapping_common.entity
from mapping_common.map import Map
from mapping_common.entity import FlagFilter, Entity
from mapping_common.markers import debug_marker, debug_marker_array
from mapping_common.transform import Vector2, Point2, Transform2D
from mapping_interfaces.msg import Map as MapMsg

from planning_interfaces.srv import SpeedAlteration

MARKER_NAMESPACE: str = "acc"
ACC_MARKER_COLOR = (0, 1.0, 1.0, 0.5)


class ACC(Node):
    """ACC (Adaptive Cruise Control) calculates and publishes the desired speed based on
    possible collisions, the current speed, the trajectory, and the speed limits."""

    map: Optional[Map] = None

    trajectory_local: Optional[Path] = None
    __curr_behavior: Optional[str] = None
    speed_limit: float = 5.0
    external_speed_limit: Optional[float] = None
    speed_override: Optional[float] = None
    steer: Optional[float] = None
    last_desired_speed: float = 0.0
    emergency_count: int = 0

    def __init__(self):
        super().__init__("ACC")
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Parameters
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )

        self.k_p = (
            self.declare_parameter(
                "k_p",
                0.5,
                descriptor=ParameterDescriptor(
                    description="Kp used for the PI controller",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=3.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.k_i = (
            self.declare_parameter(
                "k_i",
                1.2,
                descriptor=ParameterDescriptor(
                    description="Ki used for the PI controller",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=3.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.t_gap = (
            self.declare_parameter(
                "t_gap",
                1.9,
                descriptor=ParameterDescriptor(
                    description="Time gap used for the PI controller",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=5.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.d_min = (
            self.declare_parameter(
                "d_min",
                0.7,
                descriptor=ParameterDescriptor(
                    description="Minimal distance to the object in front when standing",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )

        self.hard_approach_distance = (
            self.declare_parameter(
                "hard_approach_distance",
                1.5,
                descriptor=ParameterDescriptor(
                    description="Minimum distance when closely approaching an obstacle",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.hard_approach_speed = (
            self.declare_parameter(
                "hard_approach_speed",
                1.0,
                descriptor=ParameterDescriptor(
                    description="Minimum speed when closely approaching an obstacle",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )

        self.acceleration_factor = (
            self.declare_parameter(
                "acceleration_factor",
                1.0,
                descriptor=ParameterDescriptor(
                    description="Adjusts the acceleration",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=2.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )

        self.curve_line_angle = (
            self.declare_parameter(
                "curve_line_angle",
                15.0,
                descriptor=ParameterDescriptor(
                    description="Angle (deg!) of the line used to calculate the curve distance",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=90.0, step=0.1)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.min_curve_speed = (
            self.declare_parameter(
                "min_curve_speed",
                4.0,
                descriptor=ParameterDescriptor(
                    description="Minimum desired curve speed at min_curve_distance",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=5.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.min_curve_distance = (
            self.declare_parameter(
                "min_curve_distance",
                2.0,
                descriptor=ParameterDescriptor(
                    description="Distance to the intersection with the trajectory",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=10.0, step=0.01)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.max_curve_speed = (
            self.declare_parameter(
                "max_curve_speed",
                30.0,
                descriptor=ParameterDescriptor(
                    description="Maximum desired curve speed at max_curve_distance",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=50.0, step=0.1)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )
        self.max_curve_distance = (
            self.declare_parameter(
                "max_curve_distance",
                50.0,
                descriptor=ParameterDescriptor(
                    description="Adjusts the acceleration",
                    floating_point_range=[
                        FloatingPointRange(from_value=0.0, to_value=200.0, step=0.1)
                    ],
                ),
            )
            .get_parameter_value()
            .double_value
        )

        # Get Map
        self.map_sub: Subscription = self.create_subscription(
            MapMsg,
            f"/paf/{self.role_name}/mapping/init_data",
            self.__set_map,
            qos_profile=1,
        )

        # Get current speed limit
        self.speed_limit_sub: Subscription = self.create_subscription(
            Float32,
            f"/paf/{self.role_name}/speed_limit",
            self.__set_speed_limit,
            qos_profile=1,
        )

        # Get trajectory for collision mask calculation
        self.trajectory_local_sub: Subscription = self.create_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory_local",
            self.__set_trajectory_local,
            qos_profile=1,
        )

        # Get current behavior
        self.curr_behavior_sub: Subscription = self.create_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1,
        )

        # Get current steering to adjust the front collision mask
        self.steer_sub: Subscription = self.create_subscription(
            Float32,
            f"/paf/{self.role_name}/pure_pursuit_steer",
            self.__set_steer,
            qos_profile=1,
        )

        # Publish desired speed to acting
        self.velocity_pub: Publisher = self.create_publisher(
            Float32, f"/paf/{self.role_name}/acc_velocity", qos_profile=1
        )

        # Publish to emergency break if needed
        self.emergency_pub = self.create_publisher(
            Bool,
            f"/paf/{self.role_name}/emergency",
            qos_profile=QoSProfile(
                depth=10, durability=DurabilityPolicy.TRANSIENT_LOCAL
            ),
        )

        # Publish debugging marker
        self.marker_publisher: Publisher = self.create_publisher(
            MarkerArray, f"/paf/{self.role_name}/acc/debug_markers", qos_profile=1
        )

        self.speed_service = self.create_service(
            SpeedAlteration,
            f"/paf/{self.role_name}/acc/speed_alteration",
            self.handle_speed_alteration,
        )

        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def __set_speed_limit(self, data: Float32):
        self.speed_limit = data.data

    def __set_map(self, data: MapMsg):
        self.map = Map.from_ros_msg(data)
        self.update_velocity()

    def __set_trajectory_local(self, data: Path):
        """Receive trajectory from motion planner

        Args:
            data (Path): Trajectory path
        """
        self.trajectory_local = data

    def __set_curr_behavior(self, data: String):
        """Receive the current behavior of the vehicle.

        Args:
            data (String): Current behavior
        """
        self.__curr_behavior = data.data

    def __set_steer(self, data: Float32):
        """Receive the steering angle of the vehicle.

        Args:
            data (Float32): Current steering angle
        """
        self.steer = data.data

    def update_velocity(self):
        """
        Permanent checks if distance to a possible object is too small and
        publishes the desired speed to motion planning
        """
        if self.map is None or self.trajectory_local is None:
            # We don't have the necessary data to drive safely
            self.velocity_pub.publish(Float32(data=0.0))
            return

        hero = self.map.hero()
        if hero is None or hero.motion is None:
            # We currenly have no hero data.
            # -> cannot drive safely
            self.get_logger().error("ACC: No hero with motion found in map!")
            self.velocity_pub.publish(Float32(data=0.0))
            return
        hero_width = max(1.0, hero.get_width())

        def filter_fn(e: Entity) -> bool:
            filter_collision = FlagFilter(is_collider=True, is_hero=False)
            filter_stopmark = FlagFilter(is_stopmark=True)
            return e.matches_filter(filter_collision) or e.matches_filter(
                filter_stopmark
            )

        tree = self.map.build_tree(filter_fn=filter_fn)

        if self.__curr_behavior is not None and self.__curr_behavior.startswith("ot_"):
            front_mask_size = 3.5
        else:
            front_mask_size = 5.5

        collision_masks = mapping_common.mask.build_lead_vehicle_collision_masks(
            Point2.new(hero.get_front_x(), 0.0),
            hero_width,
            self.trajectory_local,
            front_mask_size=front_mask_size,
            max_trajectory_check_length=100.0,
            rotate_front_mask=0.0 if self.steer is None else -self.steer,
        )

        collision_mask = shapely.union_all(collision_masks)

        debug_markers: List[Marker] = []
        for mask in collision_masks:
            debug_markers.append(debug_marker(mask, color=ACC_MARKER_COLOR))

        entity_result = tree.get_nearest_entity(collision_mask, hero.to_shapely())

        current_velocity = hero.get_global_x_velocity() or 0.0
        desired_speed: float = float("inf")
        lead_x_velocity: Optional[float] = None
        obstacle_is_stopmark = False
        marker_text: str = "ACC overview:"
        if entity_result is not None:
            entity, distance = entity_result
            debug_markers.append(
                debug_marker(entity.entity, color=(1.0, 0.0, 0.0, 0.5), scale_z=0.3)
            )

            lead_delta_velocity = (
                hero.get_delta_forward_velocity_of(entity.entity) or -current_velocity
            )
            desired_speed = self.calculate_velocity_based_on_lead(
                current_velocity, distance, lead_delta_velocity
            )

            lead_x_velocity = entity.entity.get_global_x_velocity()
            lead_x_velocity_str = (
                "None" if lead_x_velocity is None else f"{lead_x_velocity:6.4f}"
            )
            marker_text += (
                f"\nLeadDistance: {distance:7.4f}"
                + f"\nLeadXVelocity: {lead_x_velocity_str}"
                + f"\nDeltaV: {lead_delta_velocity:6.4f}"
                + f"\nMaxLeadSpeed: {desired_speed:6.4f}"
            )
            obstacle_is_stopmark = entity.entity.flags.matches_filter(
                FlagFilter(is_stopmark=True)
            )

        # max speed is the current speed limit
        desired_speed = min(self.speed_limit, desired_speed)

        # apply the external_speed_limit
        if self.external_speed_limit:
            desired_speed = min(desired_speed, self.external_speed_limit)

        curve_speed, c_markers = self.calculate_velocity_based_on_trajectory(hero)
        desired_speed = min(curve_speed, desired_speed)
        marker_text += f"\nMaxCurveSpeed: {curve_speed:6.4f}"

        debug_markers.extend(c_markers)

        if desired_speed == curve_speed:
            speed_reason = "Curve"
        elif desired_speed == self.speed_limit:
            speed_reason = "Speed limit"
        elif (
            self.external_speed_limit is not None
            and desired_speed == self.external_speed_limit
        ):
            speed_reason = "External speed limit"
        else:
            speed_reason = "Obstacle"

        if self.speed_override is not None:
            desired_speed = self.speed_override
            speed_reason = "Override"

        marker_text += f"\nFinalACCSpeed: {desired_speed:6.4f}"
        marker_text += f"\nSpeed reason: {speed_reason}"

        # emergency break if obstacle and difference to last desired speed is too big
        # and we are driving fast and obstacle is slow
        speed_diff = self.last_desired_speed - desired_speed
        hero_speed = hero.motion.linear_motion.x()
        slow_obstacle = True
        if lead_x_velocity is not None and abs(lead_x_velocity) > 3.0:
            slow_obstacle = False

        if slow_obstacle and not obstacle_is_stopmark and hero_speed > 7.0:
            if self.emergency_count == 0:
                if speed_diff > 7.0:
                    self.emergency_count += 1
            elif self.emergency_count < 5:
                if speed_diff > -1.0:
                    self.emergency_count += 1
                else:
                    self.emergency_count = 0
            else:
                self.emergency_pub.publish(Bool(data=True))
                self.emergency_count = 0
            marker_text += f"\nEmergency count: {self.emergency_count}/5"
        else:
            self.emergency_count = 0
        # set last desired speed to current desired speed for next loop
        self.last_desired_speed = desired_speed

        debug_markers.append(
            debug_marker(
                marker_text,
                color=(1.0, 1.0, 1.0, 1.0),
                offset=Vector2.new(-2.0, 0.0),
            )
        )

        self.velocity_pub.publish(desired_speed)

        marker_array = debug_marker_array(
            MARKER_NAMESPACE, debug_markers, self.get_clock().now().to_msg()
        )
        self.marker_publisher.publish(marker_array)

    def calculate_velocity_based_on_lead(
        self, hero_velocity: float, lead_distance: float, delta_v: float
    ) -> float:
        """Calculates the desired speed based on the distance and speed of the leading
           vehicle using a PI controller.

        Args:
            hero_velocity (float): Own velocity (of the hero vehicle)
            lead_distance (float): Distance to the leading vehicle
            delta_v (float): Difference between velocity of the leading vehicle and own
                velocity (negative if the own vehicle is faster than the leading
                vehicle)

        Returns:
            float: Desired speed
        """
        desired_speed: float = float("inf")
        d_min: float = self.d_min

        # PI controller which chooses the desired speed
        Kp: float = self.k_p
        Ki: float = self.k_i
        T_gap: float = self.t_gap
        acceleration_factor: float = self.acceleration_factor

        desired_distance = d_min + T_gap * hero_velocity
        delta_d = lead_distance - desired_distance
        speed_adjustment = Ki * delta_d + Kp * delta_v
        # we want to accelerate more slowly
        if hero_velocity > 1 and speed_adjustment > 0:
            speed_adjustment *= acceleration_factor
        desired_speed = hero_velocity + speed_adjustment

        # Use at least hard_approach_speed until hard_approach_distance is reached
        if lead_distance - self.hard_approach_distance > 0:
            desired_speed = max(desired_speed, self.hard_approach_speed)

        # desired speed should not be negative, only drive forward
        desired_speed = max(desired_speed, 0.0)

        return desired_speed

    def calculate_velocity_based_on_trajectory(
        self, hero: Entity
    ) -> Tuple[float, List[Marker]]:
        """Approximates a maximum safe cornering speed.

        Traces two lines at +/- ~curve_line_angle from the front of the car
        and measures the distance at which they intersect with the trajectory.

        Based on this distance, a suitable speed is calculated.

        For parameter descriptions, look in planning/config/ACC.cfg

        Args:
            hero (Entity): Hero entity

        Returns:
            Tuple[float, List[Marker]]: (desired_speed, debug_markers)
        """
        debug_markers: List[Marker] = []

        hero_width = hero.get_width()
        hero_front_x = hero.get_front_x()
        front_point = Point2.new(hero_front_x, 0.0)
        trajectory_line = mapping_common.mask.build_trajectory_from_start(
            self.trajectory_local, front_point, max_centering_dist=None
        )

        # Creates lines that exit the car left and right at an angle
        curve_angle: float = math.radians(self.curve_line_angle)
        max_curve_distance: float = self.max_curve_distance
        curve_line_end_base = (Vector2.forward() * max_curve_distance).point()
        curve_line_angles = [curve_angle, -curve_angle]
        curve_line_offsets = [
            Vector2.new(hero_front_x, hero_width / 2),
            Vector2.new(hero_front_x, -hero_width / 2),
        ]
        curve_lines: List[shapely.LineString] = []
        for angle, offset in zip(curve_line_angles, curve_line_offsets):
            transform = Transform2D.new_rotation_translation(angle, offset)
            line_end: Point2 = transform * curve_line_end_base
            line = shapely.LineString(
                [offset.point().to_shapely(), line_end.to_shapely()]
            )
            curve_lines.append(line)
            debug_markers.append(debug_marker(line, color=ACC_MARKER_COLOR))

        # Calculate the smallest distance where the trajectory intersects with
        # any curve line
        intersection_dist = float("inf")
        intersection_point = None
        for curve_line in curve_lines:
            intersection = shapely.intersection(trajectory_line, curve_line)
            for coord in shapely.get_coordinates(intersection):
                int_point = Point2.new(coord[0], coord[1])
                int_dist = front_point.distance_to(int_point)
                if int_dist < intersection_dist:
                    intersection_point = int_point
                    intersection_dist = int_dist

        if intersection_point is None or intersection_dist == float("inf"):
            return (float("inf"), debug_markers)

        debug_markers.append(debug_marker(intersection_point, color=ACC_MARKER_COLOR))
        debug_markers.append(
            debug_marker(
                f"Curve dist: {intersection_dist:4.1f}",
                color=(1.0, 1.0, 1.0, 1.0),
                offset=intersection_point.vector(),
            )
        )

        # Linear interpolation of the desired curve speed based on the intersection_dist
        min_curve_speed: float = self.min_curve_speed
        min_curve_distance: float = self.min_curve_distance
        max_curve_speed: float = self.max_curve_speed

        intersection_dist = max(0, intersection_dist - min_curve_distance)
        max_curve_distance = max(0, max_curve_distance - min_curve_distance)
        if max_curve_distance == 0:
            self.get_logger().warn(
                "ACC: max_curve_distance should not be <= min_curve_distance",
                throttle_duration_sec=2,
            )
            return (float("inf"), debug_markers)

        speed_percentage = intersection_dist / max_curve_distance
        desired_speed = (
            speed_percentage * (max_curve_speed - min_curve_speed) + min_curve_speed
        )
        return (desired_speed, debug_markers)

    def handle_speed_alteration(
        self, req: SpeedAlteration.Request, res: SpeedAlteration.Response
    ):
        self.speed_override = (
            None if not req.speed_override_active else req.speed_override
        )
        self.external_speed_limit = (
            None if not req.speed_limit_active else req.speed_limit
        )

        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ACC()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

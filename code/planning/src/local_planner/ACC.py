#!/usr/bin/env python
import ros_compatibility as roscomp
from geometry_msgs.msg import PoseStamped, Point
import mapping_common.markers
import mapping_common.shape
from nav_msgs.msg import Path
from ros_compatibility.node import CompatibleNode
import rospy
from rospy import Publisher, Subscriber
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from visualization_msgs.msg import Marker, MarkerArray
from typing import Optional
from typing import List
from planning.cfg import ACCConfig
from dynamic_reconfigure.server import Server

import shapely

import mapping_common.map
import mapping_common.mask
import mapping_common.entity
from mapping_common.map import Map
from mapping_common.entity import FlagFilter
from mapping_common.markers import debug_marker, debug_marker_array
from mapping_common.transform import Vector2
from mapping.msg import Map as MapMsg

MARKER_NAMESPACE: str = "acc"


class ACC(CompatibleNode):
    """ACC (Adaptive Cruise Control) calculates and publishes the desired speed based on
    possible collisions, the current speed, the trajectory, and the speed limits."""

    map: Optional[Map] = None

    # unstuck attributes
    __unstuck_flag: bool = False
    __unstuck_distance: float = -1
    # List of all speed limits, sorted by waypoint index
    __speed_limits_OD: List[float] = []
    trajectory_global: Optional[Path] = None
    trajectory: Optional[Path] = None
    __current_position: Optional[Point] = None
    # Current index from waypoint
    __current_wp_index: int = 0
    __current_heading: Optional[float] = None
    __curr_behavior: Optional[str] = None
    speed_limit: Optional[float] = None  # m/s

    def __init__(self):
        super(ACC, self).__init__("ACC")
        self.role_name = self.get_param("role_name", "hero")

        # Get Map
        self.map_sub: Subscriber = self.new_subscription(
            MapMsg,
            f"/paf/{self.role_name}/mapping/init_data",
            self.__get_map,
            qos_profile=1,
        )

        # Get Unstuck flag and distance for unstuck routine
        self.unstuck_flag_sub: Subscriber = self.new_subscription(
            Bool,
            f"/paf/{self.role_name}/unstuck_flag",
            self.__get_unstuck_flag,
            qos_profile=1,
        )
        self.unstuck_distance_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/unstuck_distance",
            self.__get_unstuck_distance,
            qos_profile=1,
        )

        # Get initial set of speed limits from global planner
        self.speed_limit_OD_sub: Subscriber = self.new_subscription(
            Float32MultiArray,
            f"/paf/{self.role_name}/speed_limits_OpenDrive",
            self.__set_speed_limits_opendrive,
            qos_profile=1,
        )

        # Get global trajectory to determine current speed limit
        self.trajectory_global_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory_global",
            self.__set_trajectory_global,
            qos_profile=1,
        )

        # Get trajectory for collision mask calculation
        self.trajectory_local_sub: Subscriber = self.new_subscription(
            Path,
            f"/paf/{self.role_name}/trajectory_local",
            self.__set_trajectory_local,
            qos_profile=1,
        )

        # Get current position to determine current waypoint
        self.pose_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic=f"/paf/{self.role_name}/current_pos",
            callback=self.__current_position_callback,
            qos_profile=1,
        )

        # Get current_heading
        self.heading_sub: Subscriber = self.new_subscription(
            Float32,
            f"/paf/{self.role_name}/current_heading",
            self.__get_heading,
            qos_profile=1,
        )

        # Get current behavior
        self.curr_behavior_sub: Subscriber = self.new_subscription(
            String,
            f"/paf/{self.role_name}/curr_behavior",
            self.__set_curr_behavior,
            qos_profile=1,
        )

        # Publish desired speed to acting
        self.velocity_pub: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/acc_velocity", qos_profile=1
        )

        # Publish current waypoint and speed limit
        self.wp_publisher: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/current_wp", qos_profile=1
        )
        self.speed_limit_publisher: Publisher = self.new_publisher(
            Float32, f"/paf/{self.role_name}/speed_limit", qos_profile=1
        )

        # Publish debugging marker
        self.marker_publisher: Publisher = self.new_publisher(
            MarkerArray, f"/paf/{self.role_name}/acc/debug_markers", qos_profile=1
        )

        # Tunable values for the controllers
        self.Kp: float
        self.Ki: float
        self.T_gap: float
        self.d_min: float
        self.acceleration_factor: float
        Server(ACCConfig, self.dynamic_reconfigure_callback)

        self.logdebug("ACC initialized")

    def __get_map(self, data: MapMsg):
        self.map = Map.from_ros_msg(data)
        self.update_velocity()

    def dynamic_reconfigure_callback(self, config: "ACCConfig", level):
        self.Kp = config["Kp"]
        self.Ki = config["Ki"]
        self.T_gap = config["T_gap"]
        self.d_min = config["d_min"]
        self.acceleration_factor = config["acceleration_factor"]

        return config

    def __get_unstuck_flag(self, data: Bool):
        """Set unstuck flag

        Args:
            data (Bool): Unstuck flag
        """
        self.__unstuck_flag = data.data

    def __get_unstuck_distance(self, data: Float32):
        """Set unstuck distance

        Args:
            data (Float32): Unstuck distance
        """
        self.__unstuck_distance = data.data

    def __get_heading(self, data: Float32):
        """Receive current heading

        Args:
            data (Float32): Current heading
        """
        self.__current_heading = float(data.data)

    def __set_trajectory_global(self, data: Path):
        """Recieve trajectory from global planner

        Args:
            data (Path): Trajectory path
        """
        self.trajectory_global = data

    def __set_trajectory_local(self, data: Path):
        """Receive trajectory from motion planner

        Args:
            data (Path): Trajectory path
        """
        self.trajectory = data

    def __set_curr_behavior(self, data: String):
        """Receive the current behavior of the vehicle.

        Args:
            data (String): Current behavior
        """
        self.__curr_behavior = data.data

    def __set_speed_limits_opendrive(self, data: Float32MultiArray):
        """Recieve speed limits from OpenDrive via global planner

        Args:
            data (Float32MultiArray): speed limits per waypoint
        """
        self.__speed_limits_OD = data.data

    def __current_position_callback(self, data: PoseStamped):
        """Get current position and check if next waypoint is reached
            If yes -> update current waypoint and speed limit

        Args:
            data (PoseStamped): Current position from perception
        """
        self.__current_position = data.pose.position
        if len(self.__speed_limits_OD) < 1 or self.trajectory_global is None:
            return

        agent = self.__current_position
        # Get current waypoint
        current_wp = self.trajectory_global.poses[self.__current_wp_index].pose.position
        # Get next waypoint
        next_wp = self.trajectory_global.poses[
            self.__current_wp_index + 1
        ].pose.position
        # distances from agent to current and next waypoint
        d_old = abs(agent.x - current_wp.x) + abs(agent.y - current_wp.y)
        d_new = abs(agent.x - next_wp.x) + abs(agent.y - next_wp.y)
        if d_new < d_old:
            # If distance to next waypoint is smaller than to current
            # update current waypoint and corresponding speed limit
            self.__current_wp_index += 1
            self.wp_publisher.publish(self.__current_wp_index)
            self.speed_limit = self.__speed_limits_OD[self.__current_wp_index]
            self.speed_limit_publisher.publish(self.speed_limit)
        # in case we used the unstuck routine to drive backwards
        # we have to follow WPs that are already passed
        elif self.__unstuck_flag:
            if self.__unstuck_distance is None or self.__unstuck_distance == -1:
                return
            self.__current_wp_index -= int(self.__unstuck_distance)
            self.wp_publisher.publish(self.__current_wp_index)
            self.speed_limit = self.__speed_limits_OD[self.__current_wp_index]
            self.speed_limit_publisher.publish(self.speed_limit)

    def run(self):
        """
        Control loop
        :return:
        """

        self.spin()

    def update_velocity(self):
        """
        Permanent checks if distance to a possible object is too small and
        publishes the desired speed to motion planning
        """
        if (
            self.map is None
            or self.trajectory is None
            or self.__current_position is None
            or self.__current_heading is None
        ):
            # We don't have the necessary data to drive safely
            self.velocity_pub.publish(0)
            return

        hero = self.map.hero()
        if hero is None or hero.motion is None:
            # We currenly have no hero data.
            # -> cannot drive safely
            rospy.logerr("ACC: No hero with motion found in map!")
            self.velocity_pub.publish(0)
            return
        hero_width = max(1.0, hero.get_width())

        tree = self.map.build_tree(FlagFilter(is_collider=True, is_hero=False))
        hero_transform = mapping_common.map.build_global_hero_transform(
            self.__current_position.x,
            self.__current_position.y,
            self.__current_heading,
        )

        front_mask_reduce_behaviours = ["ot_wait_free", "ot_app_blocked", "ot_leave"]
        if self.__curr_behavior in front_mask_reduce_behaviours:
            front_mask_size = 5.5
        else:
            front_mask_size = 7.5

        trajectory_mask = mapping_common.mask.build_trajectory_shape(
            self.trajectory,
            hero_transform,
            start_dist_from_hero=front_mask_size,
            max_length=100.0,
            current_wp_idx=0,
            max_wp_count=None,
            centered=True,
            width=hero_width,
        )
        if trajectory_mask is None:
            # We currently have no valid path to check for collisions.
            # -> cannot drive safely
            rospy.logerr("ACC: Unable to build collision mask!")
            self.velocity_pub.publish(0)
            return

        # Add small area in front of car to the collision mask
        front_rect = mapping_common.mask.project_plane(
            front_mask_size, size_y=hero_width
        )
        collision_masks = [front_rect, trajectory_mask]
        collision_mask = shapely.union_all(collision_masks)

        debug_markers: List[Marker] = []
        for mask in collision_masks:
            debug_markers.append(debug_marker(mask, color=(0, 1.0, 1.0, 0.5)))

        entity_result = tree.get_nearest_entity(collision_mask, hero.to_shapely())

        current_velocity = hero.get_global_x_velocity() or 0.0
        desired_speed: float = float("inf")
        marker_text: str = ""
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
                f"LeadDistance: {distance:7.4f}\n"
                + f"LeadXVelocity: {lead_x_velocity_str}\n"
                + f"DeltaV: {lead_delta_velocity:6.4f}\n"
                + f"RawACCSpeed: {desired_speed:6.4f}\n"
            )

        if self.speed_limit is None:
            # if no speed limit is available, drive 5 m/s max
            desired_speed = min(5.0, desired_speed)
        else:
            # max speed is the current speed limit
            desired_speed = min(self.speed_limit, desired_speed)

        marker_text += f"FinalACCSpeed: {desired_speed:6.4f}\n"
        debug_markers.append(
            debug_marker(
                marker_text,
                color=(1.0, 1.0, 1.0, 1.0),
                offset=Vector2.new(-2.0, 0.0),
            )
        )

        self.velocity_pub.publish(desired_speed)

        marker_array = debug_marker_array(MARKER_NAMESPACE, debug_markers)
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
        d_min = self.d_min

        # if we want to overtake, we need to keep some distance to the obstacle
        if (
            self.__curr_behavior == "ot_wait_free"
            and delta_v < 2
            and lead_distance < 6 * d_min
        ):
            desired_speed = 0.0
            return desired_speed

        if hero_velocity < 2 and lead_distance < (
            d_min + 2
        ):  # approaches the leading vehicle slowly until a distance of d_min
            desired_speed = (lead_distance - d_min) / 4

        else:
            # PI controller which chooses the desired speed
            Kp = self.Kp
            Ki = self.Ki
            T_gap = self.T_gap
            d_min = self.d_min

            desired_distance = d_min + T_gap * hero_velocity
            delta_d = lead_distance - desired_distance
            speed_adjustment = Ki * delta_d + Kp * delta_v
            # we want to accelerate more slowly
            if hero_velocity > 1 and speed_adjustment > 0:
                speed_adjustment *= self.acceleration_factor
            desired_speed = hero_velocity + speed_adjustment

        # desired speed should not be negative, only drive forward
        desired_speed = max(desired_speed, 0.0)

        return desired_speed


if __name__ == "__main__":
    """
    main function starts the ACC node
    :param args:
    """
    roscomp.init("ACC")

    try:
        node = ACC()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

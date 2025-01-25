#!/usr/bin/env python
from ros_compatibility.node import CompatibleNode

from mapping.msg import Map as MapMsg
from nav_msgs.msg import Path
from std_msgs.msg import Float32

from mapping_common.entity import Entity
from mapping_common.map import Map
from mapping_common.transform import Transform2D, Point2

from rospy import Publisher, Subscriber
import ros_compatibility as roscomp
import rospy
import tf.transformations
from visualization_msgs.msg import Marker

import math
import time

import tf2_ros
import tf
from tf import transformations as t
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped, Point
from visualization_msgs.msg import MarkerArray

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
import numpy as np
from PIL import Image

import cv2

from acting.helper_functions import generate_path_from_trajectory


# PARAMETERS
DISTANCE_THRESHOLD_X: int = rospy.get_param("potential_field_distance_threshold_X", 10)
DISTANCE_THRESHOLD_Y: int = rospy.get_param("potential_field_distance_threshold_Y", 5)
RESOLUTION_SCALE: int = rospy.get_param("potential_field_resolution_scale", 10)
FORCE_FACTOR: int = rospy.get_param("potential_field_force_factor", 15)
LOCAL_TRAJECTORY_ATTR_FACTOR: int = rospy.get_param(
    "potential_field_attraction_factor", 2
)
SLOPE: int = rospy.get_param("potential_field_slope", 255)
K_VALUE: float = rospy.get_param("potential_field_K", 0.009)
MAX_GRADIENT_DESCENT_STEPS: int = rospy.get_param(
    "potential_field_max_gradient_descent_steps", 100
)
GRADIENT_FACTOR: int = rospy.get_param("potential_field_gradient_factor", 6)


class Potential_field_node(CompatibleNode):

    def __init__(self):
        self.entities: list[Entity] = []
        self.potential_field_trajectory = Path()
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.hero_transform = None
        self.trajectory = None
        self.hero_pos = None
        self.hero_heading = None
        self.listener = tf.TransformListener()
        self.local_trajectory = None
        self.__local_trajectory_matrix = None

        self.transformer = tf.Transformer()
        rospy.sleep(1)

        self.last_pub_time = rospy.get_time()

        # open up entity matrix but just looking forward
        self.entity_matrix = np.zeros(
            (
                DISTANCE_THRESHOLD_X * RESOLUTION_SCALE,
                2 * DISTANCE_THRESHOLD_Y * RESOLUTION_SCALE,
            )
        )

        self.entity_matrix_midpoint = (
            0,
            DISTANCE_THRESHOLD_Y * RESOLUTION_SCALE,
        )

        # ROS SETUP ###
        self.entities_sub: Subscriber = self.new_subscription(
            msg_type=MapMsg,
            topic="/paf/hero/mapping/init_data",
            callback=self.__get_entities,
            qos_profile=1,
        )

        self.entities_plot_pub: Publisher = self.new_publisher(
            Marker, "/paf/hero/mapping/entities_plot", 1
        )

        self.potential_field_trajectory_pub: Publisher = self.new_publisher(
            Path, "/paf/hero/potential_field_trajectory", 1
        )

        self.potential_field_marker_pub: Publisher = self.new_publisher(
            MarkerArray, "/paf/hero/potential_field_markers", 1
        )

        self.local_trajectory_pub: Publisher = self.new_publisher(
            Path, "/paf/hero/local_trajectory", 1
        )

        self.trajectory_sub: Subscriber = self.new_subscription(
            msg_type=Path,
            topic="/paf/hero/trajectory",
            callback=self.__get_trajectory,
            qos_profile=1,
        )

        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/hero/current_pos",
            callback=self.__get_hero_pos,
            qos_profile=1,
        )

        self.current_heading_sub: Subscriber = self.new_subscription(
            msg_type=Float32,
            topic="/paf/hero/current_heading",
            callback=self.__get_hero_heading,
            qos_profile=1,
        )

        # END ROS SETUP ###

    # CALLBACKS ###
    def __get_hero_pos(self, data: PoseStamped):
        self.hero_pos = data

    def __get_hero_heading(self, data: Float32):
        self.hero_heading = data

    def __get_entities(self, data: MapMsg):
        """
        Callback for the entities subscriber
        :param data: The received data
        """
        self.map = Map.from_ros_msg(data)
        self.map.to_multi_poly_array(
            (
                DISTANCE_THRESHOLD_X * RESOLUTION_SCALE,
                DISTANCE_THRESHOLD_Y * 2 * RESOLUTION_SCALE,
            ),
            RESOLUTION_SCALE,
            self,
        )
        self.entities = self.map.entities_without_hero()

    def __get_trajectory(self, data: Path):
        self.trajectory = data

    # END CALLBACKS ###

    def __check_in_potential_field_horizon(self, pose: PoseStamped) -> bool:
        position = pose.pose.position
        if position.x < 0 or position.x > DISTANCE_THRESHOLD_X:
            return False
        if position.y < -DISTANCE_THRESHOLD_Y or position.y > DISTANCE_THRESHOLD_Y:
            return False
        return True

    def __filter_entities(self):
        """
        Filters the entities and fills the entity matrix
        """
        self.entity_matrix = np.zeros(
            (
                DISTANCE_THRESHOLD_X * RESOLUTION_SCALE,
                2 * DISTANCE_THRESHOLD_Y * RESOLUTION_SCALE,
            )
        )
        # fill in the entities into a matrix
        for entity in self.entities:
            # try to filter out the car entities
            if (
                abs(entity.transform.translation().x()) < 3.5
                and abs(entity.transform.translation().y()) < 2
            ):
                continue
            x = (
                int(entity.transform.translation().x() * RESOLUTION_SCALE)
                + self.entity_matrix_midpoint[0]
            )
            y = (
                int(entity.transform.translation().y() * RESOLUTION_SCALE)
                + self.entity_matrix_midpoint[1]
            )

            try:
                if self.entity_matrix[x][y] == 0:
                    self.entity_matrix[x][y] += 1
            except IndexError:
                # if the index not in the map horizon, skip the entity
                continue

    def __get_local_trajectory(self):

        if self.trajectory is None or self.hero_pos is None:
            return

        self.loginfo(f"transforming trajectory with {len(self.trajectory.poses)}")

        trans2d = Transform2D.new_rotation(-self.hero_heading.data + math.pi / 2)

        local_traj = Path()
        local_traj.header = rospy.Header()
        local_traj.header.stamp = rospy.Time.now()
        local_traj.header.frame_id = "hero"
        local_traj.poses = []
        for pose in self.trajectory.poses:
            new_pose = PoseStamped()
            new_pose.header.frame_id = "hero"

            pose_point = Point2.new(
                -(pose.pose.position.y - self.hero_pos.pose.position.y),
                pose.pose.position.x - self.hero_pos.pose.position.x,
            )
            pose_point: Point2 = trans2d * pose_point
            new_pose.pose.position.y = pose_point.y()
            new_pose.pose.position.x = pose_point.x()

            new_pose.pose.position.z = 0
            new_pose.pose.orientation.x = 0
            new_pose.pose.orientation.y = 0
            new_pose.pose.orientation.z = 0
            new_pose.pose.orientation.w = 1

            if self.__check_in_potential_field_horizon(new_pose):
                local_traj.poses.append(new_pose)

        self.loginfo(f"transformed trajectory with {len(local_traj.poses)}")

        self.local_trajectory_pub.publish(local_traj)

        self.local_trajectory = local_traj

    def __generate_local_trajectory_matrix(self):
        if self.local_trajectory is None:
            return

        matrix = np.zeros(
            (
                DISTANCE_THRESHOLD_X * RESOLUTION_SCALE,
                2 * DISTANCE_THRESHOLD_Y * RESOLUTION_SCALE,
            )
        )
        for i in range(len(self.local_trajectory.poses) - 1):
            pt1: cv2.typing.Point = (
                int(self.local_trajectory.poses[i].pose.position.x * RESOLUTION_SCALE),
                int(self.local_trajectory.poses[i].pose.position.y * RESOLUTION_SCALE),
            )
            pt2: cv2.typing.Point = (
                int(
                    self.local_trajectory.poses[i + 1].pose.position.x
                    * RESOLUTION_SCALE
                ),
                int(
                    self.local_trajectory.poses[i + 1].pose.position.y
                    * RESOLUTION_SCALE
                ),
            )

            # Use cv2.line to draw a line between consecutive points
            cv2.line(matrix, pt1, pt2, color=255, thickness=1)

        self.__local_trajectory_matrix = matrix

    def publish_potential_field_markers(self, matrix):
        marker_array = MarkerArray()
        for index, value in np.ndenumerate(matrix):
            position = Point(
                index[0] / RESOLUTION_SCALE, index[1] / RESOLUTION_SCALE, value
            )
            marker = Marker()
            marker.header.frame_id = "hero"
            marker.header.stamp = rospy.Time.now()
            marker.ns = "markers"
            marker.type = Marker.SPHERE  # You can choose other marker types
            marker.action = Marker.ADD
            marker.pose.position = position
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            marker.color.a = 1.0  # Alpha (transparency)
            marker.color.r = 1.0  # Red
            marker.color.g = 0.0  # Green
            marker.color.b = 0.0  # Blue
            marker_array.markers.append(marker)

        self.potential_field_marker_pub.publish(marker_array)
        self.loginfo("potential_field_markers published")

    def __calculate_field(self):
        """
        Calculates the potential field and publishes the trajectory
        """
        if self.entity_matrix is None:
            return

        starttime = time.time()

        self.__generate_local_trajectory_matrix()

        if self.__local_trajectory_matrix is not None:
            local_traj_matrix = self.__local_trajectory_matrix
        else:
            local_traj_matrix = np.zeros(self.entity_matrix.shape)

        # normalize the entity matrix to 0-255
        if np.max(self.entity_matrix) != 0:
            self.entity_matrix = self.entity_matrix / np.max(self.entity_matrix) * 255

        distances: np.ndarray = distance_transform_edt(self.entity_matrix == 0)

        # INTRODUCE A FORCE POINTING TO THE TOP OF THE IMAGE (TO MAKE THE CAR DRIVE)
        # slope distances matrix to the top of the image
        slope_array: np.ndarray = np.linspace(SLOPE, 0, self.entity_matrix.shape[0])
        # make slope array same shape as distances
        slope_array = np.tile(slope_array, (self.entity_matrix.shape[1], 1)).T
        distances += slope_array

        local_traj_matrix: np.ndarray = distance_transform_edt(local_traj_matrix == 0)
        local_traj_matrix = local_traj_matrix * LOCAL_TRAJECTORY_ATTR_FACTOR
        pot_field_matrix = distances - local_traj_matrix

        # plot the updated trajectory
        # smooth the values with exponential decay
        smoothed_values = np.exp(-K_VALUE * pot_field_matrix)
        # normalize smoothed values to 0-255
        if np.max(smoothed_values) != 0:
            smoothed_values = smoothed_values / np.max(smoothed_values) * 255

        # self.publish_potential_field_markers(smoothed_values)

        gradient_x, gradient_y = np.gradient(smoothed_values)

        # GRADIENT DESCENT
        finished = False
        num_steps = 0
        points: list[tuple[float]] = []
        plot_points: list[tuple[int]] = []
        min_gradient_magnitude = 1e-4

        x, y = self.entity_matrix_midpoint
        while not finished and num_steps < MAX_GRADIENT_DESCENT_STEPS:
            # move along the gradient
            try:
                dx = gradient_x[x, y] * GRADIENT_FACTOR
                dy = gradient_y[x, y] * GRADIENT_FACTOR
                # gradient_magnitude = np.sqrt(dx * dx + dy * dy)
                # if gradient_magnitude < min_gradient_magnitude:
                #    finished = True
                #    break
                x -= int(dx)
                y -= int(dy)
                plot_points.append((x, y))
                points.append(
                    (
                        # back to the original coordinates
                        -(x - self.entity_matrix_midpoint[0]) / RESOLUTION_SCALE,
                        -(y - self.entity_matrix_midpoint[1]) / RESOLUTION_SCALE,
                    )
                )
                # Allow some backwards movement but limit it
                if len(points) > 2 and points[-1][0] < points[-2][0] - 0.5:
                    self.loginfo("Excessive backwards movement detected")
                    finished = True
                num_steps += 1
            except IndexError:
                self.loginfo(f"index out of bounds after {num_steps} steps, way found")
                finished = True

        if num_steps >= MAX_GRADIENT_DESCENT_STEPS:
            self.loginfo(
                f"Warning: Maximum steps reached without convergence, last gradient: {dx,dy}"
            )
        # generate a path from the trajectory and publish
        if (
            points[-1][0] > 2
        ):  # only publish if its usable (sometime gets stuck around the origin)
            self.potential_field_trajectory = generate_path_from_trajectory(points)
            self.potential_field_trajectory_pub.publish(self.potential_field_trajectory)
        self.loginfo(f"POTENTIAL_FIELD_PUBLISHED after time {time.time()-starttime}")

    def run(self):
        """
        Runs the potential field node
        """
        self.loginfo("Potential Field Node Running")

        def loop(timerevent=None):

            self.__filter_entities()

            self.__calculate_field()

            self.__get_local_trajectory()
            # PLOTTING
            # self.save_image(
            #    self.entity_matrix_for_plotting, "/workspace/code/entity_matrix.png"
            # )

        self.new_timer(0.5, loop)
        self.spin()


def main(args=None):
    roscomp.init("potential_field_node", args=args)

    try:
        node = Potential_field_node()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()


if __name__ == "__main__":
    main()

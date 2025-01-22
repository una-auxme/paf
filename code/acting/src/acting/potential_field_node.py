#!/usr/bin/env python
from ros_compatibility.node import CompatibleNode

from mapping.msg import Map as MapMsg
from nav_msgs.msg import Path
from std_msgs.msg import Float32

from mapping_common.entity import Entity
from mapping_common.map import Map

from rospy import Publisher, Subscriber
import ros_compatibility as roscomp
import rospy
import tf.transformations
from visualization_msgs.msg import Marker

import tf2_ros
import tf
from tf2_geometry_msgs import do_transform_pose
from geometry_msgs.msg import TransformStamped

from tf2_msgs.msg import TFMessage
from geometry_msgs.msg import PoseStamped

from scipy.spatial.transform import Rotation
from scipy.ndimage import distance_transform_edt
import numpy as np
from PIL import Image

from acting.helper_functions import generate_path_from_trajectory


# PARAMETERS
DISTANCE_THRESHOLD_X: int = rospy.get_param("potential_field_distance_threshold_X", 10)
DISTANCE_THRESHOLD_Y: int = rospy.get_param("potential_field_distance_threshold_Y", 5)
RESOLUTION_SCALE: int = rospy.get_param("potential_field_resolution_scale", 10)
FORCE_FACTOR: int = rospy.get_param("potential_field_force_factor", 30)
SLOPE: int = rospy.get_param("potential_field_slope", 255)
K_VALUE: float = rospy.get_param("potential_field_K", 0.009)
MAX_GRADIENT_DESCENT_STEPS: int = rospy.get_param(
    "potential_field_max_gradient_descent_steps", 40
)
GRADIENT_FACTOR: int = rospy.get_param("potential_field_gradient_factor", 10)


class Potential_field_node(CompatibleNode):

    def __init__(self):
        self.entities: list[Entity] = []
        self.potential_field_trajectory = Path()
        self.role_name = self.get_param("role_name", "ego_vehicle")

        self.hero_transform = None
        self.trajectory = None
        self.hero_pos = None
        self.hero_heading = None

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
        self.__calculate_hero_transform()

    def __get_hero_heading(self, data: Float32):
        self.hero_heading = data.data
        self.__calculate_hero_transform()

    def __calculate_hero_transform(self):
        """
        This function merges the heading and the position together, merges it and
        calculates the inverse so it can transform the coordinate systems from global
        to hero
        """
        if self.hero_pos is None or self.hero_heading is None:
            return
        self.hero_transform = TransformStamped()

        self.hero_transform.header.stamp = rospy.Time.now()
        self.hero_transform.header.frame_id = "global"
        self.hero_transform.child_frame_id = "hero"

        self.hero_transform.transform.translation.x = (
            0  # -self.hero_pos.pose.position.x
        )
        self.hero_transform.transform.translation.y = (
            0  # -self.hero_pos.pose.position.y
        )
        self.hero_transform.transform.translation.z = (
            0  # -self.hero_pos.pose.position.z
        )

        rotation = Rotation.from_euler("z", self.hero_heading, degrees=False).as_quat()
        self.hero_transform.transform.rotation.x = -rotation[0]
        self.hero_transform.transform.rotation.y = -rotation[1]
        self.hero_transform.transform.rotation.z = -rotation[2]
        self.hero_transform.transform.rotation.w = rotation[3]

    def __get_entities(self, data: MapMsg):
        """
        Callback for the entities subscriber
        :param data: The received data
        """
        self.map = Map.from_ros_msg(data)
        self.entities = self.map.entities_without_hero()

    def __get_trajectory(self, data: Path):
        # currently not used
        self.trajectory = data

    # END CALLBACKS ###

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
                abs(entity.transform.translation().x()) < 2
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

        if self.hero_transform is None or self.trajectory is None:
            return

        trans = self.hero_transform

        local_traj = Path()
        local_traj.header = rospy.Header()
        local_traj.header.stamp = rospy.Time.now()
        local_traj.header.frame_id = "hero"
        local_traj.poses = []
        for pose in self.trajectory.poses:
            pose.pose.position.z = 0
            local_traj.poses.append(do_transform_pose(pose, trans))
        self.local_trajectory_pub.publish(local_traj)

    def __calculate_field(self):
        """
        Calculates the potential field and publishes the trajectory
        """
        if self.entity_matrix is None:
            return

        # normalize the entity matrix to 0-255
        if np.max(self.entity_matrix) != 0:
            self.entity_matrix = self.entity_matrix / np.max(self.entity_matrix) * 255

        # flip image verically and smooth out the values
        distances = distance_transform_edt(self.entity_matrix == 0)

        # INTRODUCE A FORCE POINTING TO THE TOP OF THE IMAGE (TO MAKE THE CAR DRIVE)
        # slope distances matrix to the top of the image
        slope_array: np.ndarray = np.linspace(SLOPE, 0, self.entity_matrix.shape[0])
        # make slope array same shape as distances
        slope_array = np.tile(slope_array, (self.entity_matrix.shape[1], 1)).T
        distances += slope_array

        # plot the updated trajectory
        # smooth the values with exponential decay
        smoothed_values = np.exp(-K_VALUE * distances)
        # normalize smoothed values to 0-255
        if np.max(smoothed_values) != 0:
            smoothed_values = smoothed_values / np.max(smoothed_values) * 255
        distances[distances == 0] = smoothed_values[distances == 0]

        gradient_x, gradient_y = np.gradient(smoothed_values)

        # GRADIENT DESCENT
        finished = False
        num_steps = 0
        points: list[tuple[float]] = []
        plot_points: list[tuple[int]] = []
        min_gradient_magnitude = 1e-4  # Add convergence threshold

        x, y = self.entity_matrix_midpoint
        while not finished and num_steps < MAX_GRADIENT_DESCENT_STEPS:
            # move along the gradient
            try:
                dx = gradient_x[x, y] * GRADIENT_FACTOR
                dy = gradient_y[x, y] * GRADIENT_FACTOR
                gradient_magnitude = np.sqrt(dx * dx + dy * dy)
                if gradient_magnitude < min_gradient_magnitude:
                    self.loginfo(
                        "Potential Field: Converged: gradient magnitude below threshold"
                    )
                    finished = True
                    break

                x -= int(dx)
                y -= int(dy)
                plot_points.append((x, y))
                points.append(
                    (
                        # back to the original coordinates
                        -(x - self.entity_matrix_midpoint[0]) / RESOLUTION_SCALE,
                        (y - self.entity_matrix_midpoint[1]) / RESOLUTION_SCALE,
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
            self.loginfo("Warning: Maximum steps reached without convergence")
        # generate a path from the trajectory and publish
        self.potential_field_trajectory = generate_path_from_trajectory(points)
        self.potential_field_trajectory_pub.publish(self.potential_field_trajectory)
        self.loginfo("potential field trajectory published")

        self.last_pub_time = rospy.get_time()

        return

        # EVERYTHING HAPPENING WITH ENTYITY MATRIX FOR PLOTTING
        self.entity_matrix_for_plotting = np.zeros(
            (
                self.entity_matrix.shape[0],
                self.entity_matrix.shape[1],
                3,
            ),
            dtype=np.uint8,
        )

        self.entity_matrix_for_plotting[:, :, 0] = np.flipud(self.entity_matrix)
        self.entity_matrix_for_plotting[:, :, 2] = smoothed_values
        for point in plot_points:
            try:
                self.entity_matrix_for_plotting[point[0], point[1], 1] = 255
            except IndexError:
                continue

    def save_image(self, matrix: np.ndarray, path: str):
        """
        Saves an image from a matrix

        Args:
            matrix (np.ndarray): the matrix to save
            path (str): the path to save the image
        """
        image = Image.fromarray(matrix)
        image = image.convert("RGB")
        image.save(path)
        self.loginfo(f"Image saved at {path}")

    def run(self):
        """
        Runs the potential field node
        """
        self.loginfo("Potential Field Node Running")

        def loop(timerevent=None):

            self.__filter_entities()

            self.__calculate_field()
            try:
                self.__get_local_trajectory()
            except Exception as e:
                self.logerr(f"PF: Exception {e} caught, from POTENTIAL FIELD")
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

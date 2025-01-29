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

import tf

from geometry_msgs.msg import Point
from visualization_msgs.msg import MarkerArray


from geometry_msgs.msg import PoseStamped

from scipy.ndimage import distance_transform_edt
import numpy as np

import cv2

from acting.helper_functions import generate_path_from_trajectory


# PARAMETERS
# Define the Horizon of the potential field
# Distance threshold in the X direction for potential field calculations
DISTANCE_THRESHOLD_X: int = rospy.get_param("potential_field_distance_threshold_X", 10)
# Distance threshold in the Y direction for potential field calculations
DISTANCE_THRESHOLD_Y: int = rospy.get_param("potential_field_distance_threshold_Y", 5)
"""
Scale factor for the resolution of the potential field since the entities have meters
as units and the potential field is calculated in a grid. We need ints as indices for
the matrix, so we need to scale the resolution to have a better representation of the
entities in the environment.
"""

LOOP_RATE: float = rospy.get_param("potential_field_loop_rate", 0.4)

RESOLUTION_SCALE: int = rospy.get_param("potential_field_resolution_scale", 10)
# Factor to scale the forces in the potential field
FORCE_FACTOR: int = rospy.get_param("potential_field_force_factor", 15)
# Attraction factor for the local trajectory in the potential field
LOCAL_TRAJECTORY_ATTR_FACTOR: int = rospy.get_param(
    "potential_field_attraction_factor", 2
)
# Slope value used in potential field calculations
SLOPE: int = rospy.get_param("potential_field_slope", 255)
# K value used in potential field calculations. The K-value is used to smooth the field
K_VALUE: float = rospy.get_param("potential_field_K", 0.009)
# Maximum number of steps for gradient descent in potential field calculations
MAX_GRADIENT_DESCENT_STEPS: int = rospy.get_param(
    "potential_field_max_gradient_descent_steps", 100
)
# Factor to scale the gradient in potential field calculations
GRADIENT_FACTOR: int = rospy.get_param("potential_field_gradient_factor", 6)


class Potential_field_node(CompatibleNode):
    """
    Potential_field_node
    takes in the entities and the trajectory and calculates the potential
    field trajectory

    Args:
        CompatibleNode
    """

    def __init__(self):
        """
        Constructor for the Potential_field_node
        """
        # Initialize the node
        self.entities: list[Entity] = []  # List to store entities in the environment
        self.potential_field_trajectory = (
            Path()
        )  # Path object to store the potential field trajectory
        self.role_name = self.get_param(
            "role_name", "ego_vehicle"
        )  # Role name of the vehicle

        # Initialize various attributes related to the vehicle's state and trajectory
        self.hero_transform = None
        self.trajectory = None
        self.hero_pos = None
        self.hero_heading = None
        self.local_trajectory = None
        self.__local_trajectory_matrix = None

        self.transformer = (
            tf.Transformer()
        )  # Transformer object for coordinate transformations
        rospy.sleep(1)  # Sleep for a second to allow the transformer to initialize

        self.last_pub_time = (
            rospy.get_time()
        )  # Store the last time a message was published

        # Initialize the entity matrix to store information about entities
        self.entity_matrix = np.zeros(
            (
                DISTANCE_THRESHOLD_X * RESOLUTION_SCALE,  # Number of rows in the matrix
                2
                * DISTANCE_THRESHOLD_Y
                * RESOLUTION_SCALE,  # Number of columns in the matrix
            )
        )

        # Midpoint of the entity matrix in the Y direction
        self.entity_matrix_midpoint = (
            0,
            DISTANCE_THRESHOLD_Y * RESOLUTION_SCALE,
        )

        # ROS SETUP ###
        # subscriber to the intermediate layer
        self.entities_sub: Subscriber = self.new_subscription(
            msg_type=MapMsg,
            topic="/paf/hero/mapping/init_data",
            callback=self.__get_entities,
            qos_profile=1,
        )

        # publisher for the entities plot
        self.entities_plot_pub: Publisher = self.new_publisher(
            Marker, "/paf/hero/mapping/entities_plot", 1
        )

        # publisher for the potential field trajectory
        self.potential_field_trajectory_pub: Publisher = self.new_publisher(
            Path, "/paf/hero/potential_field_trajectory", 1
        )

        # publisher for the potential field markers (not used currently)
        self.potential_field_marker_pub: Publisher = self.new_publisher(
            MarkerArray, "/paf/hero/potential_field_markers", 1
        )

        # publisher for the local trajectory
        self.local_trajectory_pub: Publisher = self.new_publisher(
            Path, "/paf/hero/local_trajectory", 1
        )

        # subscriber for the trajectory
        self.trajectory_sub: Subscriber = self.new_subscription(
            msg_type=Path,
            topic="/paf/hero/trajectory",
            callback=self.__get_trajectory,
            qos_profile=1,
        )

        # subscriber for the current position of the vehicle
        self.current_pos_sub: Subscriber = self.new_subscription(
            msg_type=PoseStamped,
            topic="/paf/hero/current_pos",
            callback=self.__get_hero_pos,
            qos_profile=1,
        )

        # subscriber for the current heading of the vehicle
        self.current_heading_sub: Subscriber = self.new_subscription(
            msg_type=Float32,
            topic="/paf/hero/current_heading",
            callback=self.__get_hero_heading,
            qos_profile=1,
        )

        # END ROS SETUP ###

    # CALLBACKS ###

    # get the current position of the vehicle and fill it to the hero_pos attribute
    def __get_hero_pos(self, data: PoseStamped):
        self.hero_pos = data

    # get the current heading of the vehicle and fill it to the hero_heading attribute
    def __get_hero_heading(self, data: Float32):
        self.hero_heading = data

    # get map, extract the entities and fill the entities attribute
    def __get_entities(self, data: MapMsg):
        """
        Callback for the entities subscriber
        :param data: The received data
        """
        self.map = Map.from_ros_msg(data)
        # map to multi poly array posed as a faster way to fill the entity matrix due
        # to the mapping common package being cython compiled
        # self.map.to_multi_poly_array(
        #    (
        #        DISTANCE_THRESHOLD_X * RESOLUTION_SCALE,
        #        DISTANCE_THRESHOLD_Y * 2 * RESOLUTION_SCALE,
        #    ),
        #    RESOLUTION_SCALE,
        #    self,
        # )
        self.entities = self.map.entities_without_hero()

    # get the trajectory and fill the trajectory attribute
    def __get_trajectory(self, data: Path):
        self.trajectory = data

    # END CALLBACKS ###

    def __check_in_potential_field_horizon(self, pose: PoseStamped) -> bool:
        """
        Checks if the pose is within the potential field horizon

        Args:
            pose (PoseStamped): pose to check

        Returns:
            bool: returns true if it is within the potential field horizon
        """
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

            # fill the entity matrix with the entities
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
        """
        transforms the trajectory to the "hero" coordinate system
        tried to do it with tf and tf2 but had a hard time with it.
        this version is not the most efficient but due to time constraints, it is the
        most reliable
        """

        if self.trajectory is None or self.hero_pos is None:
            return

        self.loginfo(f"transforming trajectory with {len(self.trajectory.poses)}")

        # using some parts of the transformation functionality of the mapping_common
        # package
        # we only need 2d so it is sufficient to use the 2d transformation
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
                # only add the pose if it is within the potential field horizon
                local_traj.poses.append(new_pose)

        # self.loginfo(f"transformed trajectory with {len(local_traj.poses)}")

        self.local_trajectory_pub.publish(local_traj)

        self.loginfo("published trajectory")

        self.local_trajectory = local_traj

    def __generate_local_trajectory_matrix(self):
        """
        takes the local trajectory and generates a matrix to use in the potential field
        calculations this matrix can be layered on top of the entity matrix to create a
        potential field it also can be used to tune the attractive or repulsive
        strength individually
        """
        if self.local_trajectory is None:
            return

        # generate matrix
        matrix = np.zeros(
            (
                DISTANCE_THRESHOLD_X * RESOLUTION_SCALE,
                2 * DISTANCE_THRESHOLD_Y * RESOLUTION_SCALE,
            )
        )
        # use cv2 to draw lines between the points, resulting in a matrix where indices
        # on the line are assigned the value 255
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
        """
        Publishes the potential field markers, was not used and has to be tweaked
        in order to publish the points correctly

        Args:
            matrix (np.array): the potential field matrix
        """
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
        # self.loginfo("potential_field_markers published")

    def __calculate_field(self):
        """
        Calculates the potential field and publishes the trajectory
        """
        if self.entity_matrix is None:
            return

        self.__generate_local_trajectory_matrix()

        if self.__local_trajectory_matrix is not None:
            local_traj_matrix = self.__local_trajectory_matrix
        else:
            local_traj_matrix = np.zeros(
                self.entity_matrix.shape
            )  # Initialize with zeros if not available

        # Normalize the entity matrix to 0-255
        if np.max(self.entity_matrix) != 0:
            self.entity_matrix = self.entity_matrix / np.max(self.entity_matrix) * 255

        # Calculate the distance transform of the entity matrix
        # this is used to calculate the distance to the closest entity
        distances: np.ndarray = distance_transform_edt(self.entity_matrix == 0)

        # Introduce a force pointing to the top of the image (to make the car drive)
        # Create a slope distances matrix to the top of the image
        slope_array: np.ndarray = np.linspace(SLOPE, 0, self.entity_matrix.shape[0])
        # Make slope array same shape as distances
        slope_array = np.tile(slope_array, (self.entity_matrix.shape[1], 1)).T
        distances += slope_array

        # Calculate the distance transform of the local trajectory matrix
        local_traj_matrix: np.ndarray = distance_transform_edt(local_traj_matrix == 0)
        local_traj_matrix = local_traj_matrix * LOCAL_TRAJECTORY_ATTR_FACTOR
        # subtract the local trajectory
        # matrix from the distances matrix in order to apply an attractive potential
        pot_field_matrix = distances - local_traj_matrix
        # Smooth the values with exponential decay
        smoothed_values = np.exp(-K_VALUE * pot_field_matrix)
        # Normalize smoothed values to 0-255
        if np.max(smoothed_values) != 0:
            smoothed_values = smoothed_values / np.max(smoothed_values) * 255

        # self.publish_potential_field_markers(smoothed_values)

        # Calculate the gradient of the smoothed values
        gradient_x, gradient_y = np.gradient(smoothed_values)

        # Gradient descent
        finished = False
        num_steps = 0
        points: list[tuple[float]] = []  # List to store the points of the trajectory
        plot_points: list[tuple[int]] = []  # List to store the points for plotting

        x, y = (
            self.entity_matrix_midpoint
        )  # Start from the midpoint of the entity matrix
        while not finished and num_steps < MAX_GRADIENT_DESCENT_STEPS:
            # Move along the gradient
            try:
                dx = gradient_x[x, y] * GRADIENT_FACTOR
                dy = gradient_y[x, y] * GRADIENT_FACTOR
                # gradient_magnitude = np.sqrt(dx * dx + dy * dy)
                # if gradient_magnitude < min_gradient_magnitude:
                #    finished = True
                #    break
                x -= int(dx)
                y -= int(dy)
                plot_points.append((x, y))  # Append the new point to the plot points
                points.append(
                    (x / RESOLUTION_SCALE, y / RESOLUTION_SCALE)
                )  # Append the new point to the trajectory points
                num_steps += (
                    1  # Increment the number of steps for the finished condition
                )
            except IndexError:
                # If the index is out of bounds of the potential field horizon, stop
                # the descent
                finished = True

    def run(self):
        """
        Runs the potential field node
        """
        self.loginfo("Potential Field Node Running")  # Log that the node is running

        def loop(timerevent=None):
            """
            Loop function to be called periodically
            """
            self.loginfo("potential_field: tick")
            self.__filter_entities()  # Filter the entities and update the entity matrix

            # Transform the global trajectory to the local frame
            self.__get_local_trajectory()

            # Calculate the potential field and update the trajectory
            self.__calculate_field()

        self.new_timer(LOOP_RATE, loop)  # Set a timer to call the loop function
        self.spin()  # Keep the node running


def main(args=None):
    """
    Main function to initialize and run the potential field node
    """
    roscomp.init("potential_field_node", args=args)  # Initialize the ROS node

    try:
        node = Potential_field_node()  # Create an instance of the Potential_field_node
        node.run()  # Run the node
    except KeyboardInterrupt:
        pass  # Handle keyboard interrupt
    finally:
        roscomp.shutdown()  # Shutdown the ROS node


if __name__ == "__main__":
    main()  # Run the main function if this script is executed directly

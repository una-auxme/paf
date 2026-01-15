from typing import List

from joblib import Parallel, delayed
import numpy as np
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge
from transforms3d.quaternions import mat2quat
from abc import ABC, abstractmethod

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
import rclpy.logging
import ros2_numpy
from typing import Optional

from paf_common.parameters import update_attributes
from rclpy.parameter import Parameter

from std_msgs.msg import Float32
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import Marker, MarkerArray
from mapping_interfaces.msg import ClusteredPointsArray
from carla_msgs.msg import CarlaSpeedometer
from sensor_msgs.msg import PointCloud2, Imu, Image as ImageMsg

from .perception_utils import (
    array_to_clustered_points,
    ego_motion_compensation,
    create_delta_matrix,
    create_ego_vehicle_mask,
    apply_local_motion_compensation,
    quaternion_to_heading,
)
from .lidar_filter_utility import bounding_box, remove_field_name


class LidarDistance(Node):
    """See doc/perception/lidar_distance_utility.md on
    how to configute this node
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Parameters
        self.clustering_lidar_z_min = (
            self.declare_parameter(
                "clustering_lidar_z_min",
                -1.4,
            )
            .get_parameter_value()
            .double_value
        )
        self.dbscan_eps = (
            self.declare_parameter(
                "dbscan_eps",
                0.4,
            )
            .get_parameter_value()
            .double_value
        )
        self.dbscan_min_samples = (
            self.declare_parameter(
                "dbscan_min_samples",
                10,
            )
            .get_parameter_value()
            .integer_value
        )

        self.compensation_strategy = (
            self.declare_parameter("compensation_strategy", "LocalCompensation")
            .get_parameter_value()
            .string_value
        )

        compensation_dict = {
            "NoCompensation": NoCompensation,
            "Buffer": Buffer,
            "EgoMotionCompensation": EgoMotionCompensation,
            "LocalCompensation": LocalCompensation,
        }

        self.Compensation: CompensationStrategy = compensation_dict[
            self.compensation_strategy
        ]()

        self.bridge = CvBridge()  # OpenCV bridge for image conversions

        # Publisher for filtered point clouds
        self.pub_pointcloud = self.create_publisher(
            PointCloud2,
            "/carla/hero/LIDAR_filtered",
            qos_profile=1,
        )

        # Publishers for distance images in various directions
        self.dist_array_center_publisher = self.create_publisher(
            msg_type=ImageMsg, topic="/paf/hero/Center/dist_array", qos_profile=10
        )
        self.dist_array_back_publisher = self.create_publisher(
            msg_type=ImageMsg, topic="/paf/hero/Back/dist_array", qos_profile=10
        )
        self.dist_array_left_publisher = self.create_publisher(
            msg_type=ImageMsg, topic="/paf/hero/Left/dist_array", qos_profile=10
        )
        self.dist_array_right_publisher = self.create_publisher(
            msg_type=ImageMsg, topic="/paf/hero/Right/dist_array", qos_profile=10
        )
        self.marker_visualization_lidar_publisher = self.create_publisher(
            msg_type=MarkerArray, topic="/paf/hero/Lidar/Marker", qos_profile=10
        )

        self.clustered_points_publisher = self.create_publisher(
            msg_type=ClusteredPointsArray,
            topic="/paf/hero/Lidar/clustered_points",
            qos_profile=10,
        )

        self.delta_heading_publisher = self.create_publisher(
            msg_type=Float32,
            topic="/paf/hero/delta_heading",
            qos_profile=1,
        )

        # Subscriber for LIDAR data (point clouds)
        self.create_subscription(
            msg_type=PointCloud2,
            topic="/carla/hero/LIDAR",
            callback=self.lidar_callback,
            qos_profile=10,
        )

        if self.compensation_strategy == "EgoMotionCompensation":
            self.create_subscription(
                msg_type=PoseStamped,
                topic="/paf/hero/local_current_pos",
                callback=self.ekf_callback,
                qos_profile=1,
            )

        if self.compensation_strategy == "LocalCompensation":
            self.create_subscription(
                msg_type=CarlaSpeedometer,
                topic="/carla/hero/Speed",
                callback=self.speed_callback,
                qos_profile=1,
            )

            self.create_subscription(
                msg_type=Imu,
                topic="/carla/hero/IMU",
                callback=self.imu_callback,
                qos_profile=1,
            )

        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def lidar_callback(self, data: PointCloud2):
        """
        Receives raw LIDAR point cloud data, applies motion compensation
        based on the configured strategy, and initiates downstream processing
        (clustering and image calculation) with the resulting compensated cloud.

        :param data: The raw PointCloud2 message.
        """

        self.Compensation.set_lidar_data(data)

        point_cloud = self.Compensation.compensate()
        if point_cloud is None:
            return

        if (
            isinstance(self.Compensation, LocalCompensation)
            and self.Compensation.delta_heading is not None
        ):
            msg = Float32()
            msg.data = float(self.Compensation.delta_heading)
            self.delta_heading_publisher.publish(msg)

        self.start_clustering(point_cloud)
        self.start_image_calculation(point_cloud)

    def ekf_callback(self, data: PoseStamped):
        """
        Receives EKF pose and passes it to the EgoMotionCompensation strategy.

        :param data: The local pose message (PoseStamped) containing vehicle
                     position and orientation.
        """

        if self.compensation_strategy != "EgoMotionCompensation":
            self.get_logger().warn(
                f"{type(self).__name__}"
                "EKF callback is only active for EgoMotionCompensation."
            )
            return

        self.Compensation.set_motion_data(data=data)

    def speed_callback(self, velocity: CarlaSpeedometer):
        """
        Receives velocity and passes it to the LocalCompensation strategy.

        :param velocity: The current vehicle speed message (CarlaSpeedometer).
        """

        if self.compensation_strategy != "LocalCompensation":
            self.get_logger().warn(
                f"{type(self).__name__}"
                "Speed callback is only active for LocalCompensation."
            )
            return

        self.Compensation.set_motion_data(velocity=velocity.speed)

    def imu_callback(self, imu_data: Imu):
        """
        Receives IMU data and passes the extracted heading
        to the LocalCompensation strategy.

        :param imu_data: The IMU message containing orientation data.
        """

        if self.compensation_strategy != "LocalCompensation":
            self.get_logger().warn(
                f"{type(self).__name__}"
                "IMU callback is only active for LocalCompensation."
            )
            return

        x = imu_data.orientation.x
        y = imu_data.orientation.y
        z = imu_data.orientation.z
        w = imu_data.orientation.w

        heading = quaternion_to_heading(x, y, z, w)
        self.Compensation.set_motion_data(heading=heading)

    def start_clustering(self, data):
        """
        Processes LIDAR point clouds by filtering, clustering,
        generating bounding boxes, and publishing the results.

        Steps:
        - Converts PointCloud2 data to NumPy structured array
        - Filters irrelevant LIDAR points
        - Applies DBSCAN clustering
        - Removes noise points (label = -1)
        - Generates bounding boxes for detected clusters
        - Publishes visualization markers and clustered entities

        :param data: LIDAR point clouds in ROS PointCloud2 format.
        """

        # Convert PointCloud2 data to a NumPy structured array
        coordinates = ros2_numpy.point_cloud2.pointcloud2_to_array(data)

        filtered_coordinates = coordinates[
            ~(
                (coordinates["x"] >= -2)
                & (coordinates["x"] <= 2)  # Exclude ego vehicle in x-axis
                & (coordinates["y"] >= -1)
                & (coordinates["y"] <= 1)  # Exclude ego vehicle in y-axis
            )
            & (
                coordinates["z"] > self.clustering_lidar_z_min
            )  # Exclude points below a certain height (street)
        ]

        # Perform DBSCAN clustering
        clustered_points, cluster_labels = cluster_lidar_data_from_pointcloud(
            filtered_coordinates, self.dbscan_eps, self.dbscan_min_samples
        )

        # Extract x, y, z coordinates into a separate array
        filtered_xyz = np.column_stack(
            (
                filtered_coordinates["x"],
                filtered_coordinates["y"],
                filtered_coordinates["z"],
            )
        )

        # Remove noise points (label = -1) before processing further
        valid_indices = cluster_labels != -1
        filtered_xyz = filtered_xyz[valid_indices]
        cluster_labels = cluster_labels[valid_indices]

        # Combine coordinates with their cluster labels
        points_with_labels = np.hstack((filtered_xyz, cluster_labels.reshape(-1, 1)))

        # Generate bounding boxes only for valid clusters
        if points_with_labels.size > 0:
            bounding_boxes = generate_bounding_boxes(points_with_labels)
        else:
            bounding_boxes = []

        # Create and populate a MarkerArray for visualization
        marker_array = MarkerArray()
        marker_array.markers.extend(
            create_bounding_box_marker(label, bbox) for label, bbox in bounding_boxes
        )

        # Publish visualization markers
        self.marker_visualization_lidar_publisher.publish(marker_array)

        # Prepare and publish clustered entity data
        cluster_points_np = points_with_labels[:, :3]
        index_array = points_with_labels[:, -1]
        clustered_points_msg = array_to_clustered_points(
            self.get_clock().now(),
            cluster_points_np,
            index_array,
            header_id="hero/LIDAR",
        )
        self.clustered_points_publisher.publish(clustered_points_msg)

    def start_image_calculation(self, data):
        """
        Computes distance images based on LIDAR data and publishes them.

        :param data: LIDAR point cloud in ROS PointCloud2 format.
        """
        coordinates = ros2_numpy.point_cloud2.pointcloud2_to_array(data)

        # Directions to process
        directions = ["Center", "Back", "Left", "Right"]

        # Process images for all directions
        processed_images = {
            direction: self.calculate_image(coordinates, focus=direction)
            for direction in directions
        }

        # Publish the processed images
        self.publish_images(processed_images, data.header)

    def calculate_image(self, coordinates, focus):
        """
        Calculates a distance image for a specific focus (view direction) from
        LIDAR coordinates.

        :param coordinates: Filtered LIDAR coordinates as a NumPy array.
        :param focus: The focus direction ("Center", "Back", "Left", "Right").
        :return: Distance image as a 2D array.
        """
        # Define bounding box parameters based on focus direction
        bounding_box_params = {
            "Center": {"max_x": np.inf, "min_x": 0.0, "min_z": -np.inf},
            "Back": {"max_x": 0.0, "min_x": -np.inf, "min_z": -1.6},
            "Left": {"max_y": np.inf, "min_y": 0.0, "min_z": -1.6},
            "Right": {"max_y": -0.0, "min_y": -np.inf, "min_z": -1.6},
        }

        # Select parameters for the given focus
        params = bounding_box_params.get(focus)
        if not params:
            self.get_logger().warn(
                f"Unknown focus: {focus}. Skipping image calculation."
            )
            return None

        # Apply bounding box filter
        reconstruct_bit_mask = bounding_box(coordinates, **params)
        reconstruct_coordinates = coordinates[reconstruct_bit_mask]

        # Remove the "intensity" field and convert to a NumPy array
        reconstruct_coordinates_xyz = np.array(
            remove_field_name(reconstruct_coordinates, "intensity").tolist()
        )

        if not reconstruct_coordinates_xyz.size:
            return None

        # Reconstruct the image based on the focus
        return self.reconstruct_img_from_lidar(reconstruct_coordinates_xyz, focus=focus)

    def publish_images(self, processed_images, data_header):
        """
        Publishes distance images for various directions as ROS image messages.

        :param processed_images: Dictionary with directions ("Center", "Back", etc.)
         as keys and image arrays as values.
        :param data_header: Header for the ROS image messages.
        """
        # Process only valid NumPy arrays
        for direction, image_array in processed_images.items():
            if not isinstance(image_array, np.ndarray):
                continue

            # Convert the image into a ROS image message
            dist_array_msg = self.bridge.cv2_to_imgmsg(
                image_array, encoding="passthrough"
            )
            dist_array_msg.header = data_header

            if direction == "Center":
                self.dist_array_center_publisher.publish(dist_array_msg)
            if direction == "Back":
                self.dist_array_back_publisher.publish(dist_array_msg)
            if direction == "Left":
                self.dist_array_left_publisher.publish(dist_array_msg)
            if direction == "Right":
                self.dist_array_right_publisher.publish(dist_array_msg)

    def reconstruct_img_from_lidar(self, coordinates_xyz, focus):
        """
        Reconstructs a 2D image from 3D LIDAR data for a given camera perspective.

        :param coordinates_xyz: 3D coordinates of the filtered LIDAR points.
        :param focus: Camera view (e.g., "Center", "Back").
        :return: Reconstructed image as a 2D array.
        """

        # Create the intrinsic camera matrix based on image parameters (FOV, resolution)
        im = np.identity(3)
        im[0, 2] = 1280 / 2.0  # x-offset (image center)
        im[1, 2] = 720 / 2.0  # y-offset (image center)
        im[0, 0] = im[1, 1] = 1280 / (
            2.0 * np.tan(100 * np.pi / 360.0)
        )  # Scale factor based on FOV

        # Create the extrinsic camera matrix (identity matrix for no transformation)
        ex = np.zeros(shape=(3, 4))
        ex[0][0] = ex[1][1] = ex[2][2] = 1
        m = np.matmul(im, ex)  # Combine intrinsic and extrinsic matrices

        # Initialize empty images for reconstruction
        img = np.zeros(shape=(720, 1280), dtype=np.float32)
        dist_array = np.zeros(shape=(720, 1280, 3), dtype=np.float32)

        # Prepare points based on focus
        if focus in ["Center", "Back"]:
            points = np.column_stack(
                (
                    coordinates_xyz[:, 1],
                    coordinates_xyz[:, 2],
                    coordinates_xyz[:, 0],
                    np.ones(coordinates_xyz.shape[0]),
                )
            )
        elif focus in ["Left", "Right"]:
            points = np.column_stack(
                (
                    coordinates_xyz[:, 0],
                    coordinates_xyz[:, 2],
                    coordinates_xyz[:, 1],
                    np.ones(coordinates_xyz.shape[0]),
                )
            )
        else:
            self.get_logger().warn(
                f"Unknown focus: {focus}. Skipping image calculation."
            )
            return None

        # Project 3D points to 2D image coordinates
        pixels = np.dot(m, points.T).T

        x = (pixels[:, 0] / pixels[:, 2]).astype(int)
        y = (pixels[:, 1] / pixels[:, 2]).astype(int)

        # Filter valid coordinates
        valid_indices = (x >= 0) & (x < 1280) & (y >= 0) & (y < 720)
        x = x[valid_indices]
        y = y[valid_indices]
        valid_coordinates = coordinates_xyz[valid_indices]

        if focus == "Center":
            img[719 - y, 1279 - x] = valid_coordinates[:, 0]
            dist_array[719 - y, 1279 - x] = valid_coordinates
        elif focus == "Back":
            img[y, 1279 - x] = -valid_coordinates[:, 0]
            dist_array[y, 1279 - x] = np.column_stack(
                (
                    -valid_coordinates[:, 0],
                    valid_coordinates[:, 1],
                    valid_coordinates[:, 2],
                )
            )
        elif focus == "Left":
            img[719 - y, 1279 - x] = valid_coordinates[:, 1]
            dist_array[719 - y, 1279 - x] = valid_coordinates
        elif focus == "Right":
            img[y, 1279 - x] = -valid_coordinates[:, 1]
            dist_array[y, 1279 - x] = np.column_stack(
                (
                    valid_coordinates[:, 0],
                    -valid_coordinates[:, 1],
                    valid_coordinates[:, 2],
                )
            )

        return dist_array


def generate_bounding_boxes(points_with_labels):
    """
    Generates axis-aligned bounding boxes (AABB) for clustered points.

    This function calculates bounding boxes for each unique
    cluster label in the input array.

    Args:
        points_with_labels (numpy.ndarray):
            A 2D array of shape (N, 4), where each row contains:
            - Coordinates (x, y, z) of a point
            - Cluster label in the last column
            Structure: [x, y, z, label]

    Returns:
        list:
            A list of tuples. Each tuple contains:
            - A cluster label (int/float)
            - A bounding box (tuple): (x_min, x_max, y_min, y_max, z_min, z_max)
    """
    bounding_boxes = []

    # Identify unique cluster labels
    unique_labels = np.unique(points_with_labels[:, -1])

    # Process each cluster label
    for label in unique_labels:
        if label == -1:  # Skip noise points (label = -1)
            continue

        # Extract points belonging to the current cluster
        cluster_points = points_with_labels[points_with_labels[:, -1] == label, :3]

        # Calculate the bounding box for the cluster
        bbox = calculate_aabb(cluster_points)
        bounding_boxes.append((label, bbox))

    return bounding_boxes


def create_bounding_box_marker(label, bbox, bbox_type="aabb", frame_id="hero/LIDAR"):
    """
    Creates an RViz Marker for visualizing a 3D bounding box using Marker.CUBE.

    Args:
        label (int): Unique identifier for the cluster or object.
                     Used as the Marker ID.
        bbox (tuple): Bounding box dimensions.
                      For AABB: (x_min, x_max, y_min, y_max, z_min, z_max).
                      For OBB: (center, dimensions, eigenvectors).
        bbox_type (str): The type of bounding box ("aabb" or "obb").

    Returns:
        Marker: A CUBE Marker object that can be published to RViz.
    """
    # Initialize the Marker object
    marker = Marker()
    marker.header.frame_id = frame_id  # Reference frame for the marker
    marker.ns = "marker_lidar"  # Namespace to group related markers
    marker.id = int(label)  # Use the label as the unique marker ID
    marker.lifetime = Duration(
        seconds=0.1
    ).to_msg()  # Marker visibility duration in seconds
    marker.type = Marker.CUBE  # Use a cube for the bounding box
    marker.action = Marker.ADD  # Action to add or modify the marker

    # Set marker color and opacity
    marker.color.r = 1.0  # Red
    marker.color.g = 0.5  # Green
    marker.color.b = 0.5  # Blue
    marker.color.a = 0.8  # Opacity

    if bbox_type == "aabb":
        x_min, x_max, y_min, y_max, z_min, z_max = bbox

        # Calculate center and dimensions for AABB
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        center_z = (z_min + z_max) / 2.0

        size_x = x_max - x_min
        size_y = y_max - y_min
        size_z = z_max - z_min

        marker.pose.position.x = center_x
        marker.pose.position.y = center_y
        marker.pose.position.z = center_z

        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = size_x
        marker.scale.y = size_y
        marker.scale.z = size_z

    elif bbox_type == "obb":
        center, dimensions, eigenvectors = bbox
        width, length, height = dimensions

        # Assign OBB parameters
        marker.pose.position.x = center[0]
        marker.pose.position.y = center[1]
        marker.pose.position.z = center[2]

        # Convert eigenvectors to quaternion
        quaternion = mat2quat(np.vstack([eigenvectors.T, [0, 0, 0]]).T)
        marker.pose.orientation.x = quaternion[1]
        marker.pose.orientation.y = quaternion[2]
        marker.pose.orientation.z = quaternion[3]
        marker.pose.orientation.w = quaternion[0]

        marker.scale.x = width
        marker.scale.y = length
        marker.scale.z = height

    else:
        raise ValueError(f"Unsupported bbox_type: {bbox_type}")

    return marker


def calculate_aabb(cluster_points):
    """
    Calculates the axis-aligned bounding box (AABB) for a set of 3D points.

    This function computes the minimum and maximum values along each axis (x, y, z)
    for a given set of 3D points, which defines the bounding box that contains
    all points in the cluster.

    Args:
        cluster_points (numpy.ndarray):
        A 2D array where each row represents a 3D point (x, y, z).
        The array should have shape (N, 3) where N is the number of points.

    Returns:
        tuple: A tuple of the form (x_min, x_max, y_min, y_max, z_min, z_max),
        which represents the axis-aligned bounding box (AABB) for the given
        set of points. The values are the minimum and maximum coordinates
        along the x, y, and z axes.
    """

    # for 3d boxes
    x_min = np.min(cluster_points[:, 0])
    x_max = np.max(cluster_points[:, 0])
    y_min = np.min(cluster_points[:, 1])
    y_max = np.max(cluster_points[:, 1])
    z_min = np.min(cluster_points[:, 2])
    z_max = np.max(cluster_points[:, 2])
    return x_min, x_max, y_min, y_max, z_min, z_max


def array_to_pointcloud2(
    points,
    stamp: Time,
    header="hero/Lidar",
):
    """
    Converts an array of points into a ROS PointCloud2 message.

    :param points: Array of points with [x, y, z] coordinates.
    :param header: Header information for the ROS PointCloud2 message.
    :return: ROS PointCloud2 message.
    """
    # Ensure the input is a NumPy array
    points_array = np.array(points)

    # Convert the points into a structured array with fields "x", "y", "z"
    points_structured = np.array(
        [(p[0], p[1], p[2]) for p in points_array],
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")],
    )

    # Create a PointCloud2 message from the structured array
    pointcloud_msg = ros2_numpy.point_cloud2.array_to_pointcloud2(points_structured)

    # Set the timestamp and header for the message
    pointcloud_msg.header.stamp = stamp.to_msg()
    pointcloud_msg.header = header

    return pointcloud_msg


def cluster_lidar_data_from_pointcloud(coordinates, eps, min_samples):
    """
    Performs clustering on LIDAR data using DBSCAN and returns the clusters.

    :param coordinates: LIDAR point cloud as a NumPy array with "x", "y", "z".
    :param eps: Maximum distance between points to group them into a cluster.
    :param min_samples: Minimum number of points required to form a cluster.
    :return: Dictionary with cluster IDs and their corresponding point clouds.
    """
    if coordinates.shape[0] == 0:
        rclpy.logging.get_logger("lidar_distance").warn(
            "The input array 'coordinates' is empty."
        )
        return {}

    # Extract x, y, and z from the coordinates for DBSCAN clustering
    xyz = np.column_stack((coordinates["x"], coordinates["y"], coordinates["z"]))

    if xyz.shape[0] == 0:
        rclpy.logging.get_logger("lidar_distance").warn(
            "No data points available for DBSCAN. Skipping clustering."
        )
        return {}

    # Apply DBSCAN to compute cluster labels for the point cloud
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(xyz)
    labels = clustering.labels_

    # Remove noise (cluster ID: -1) and identify valid cluster IDs
    unique_labels = np.unique(labels)
    valid_labels = unique_labels[unique_labels != -1]

    # Create a dictionary with cluster IDs and their corresponding points
    clusters = Parallel(n_jobs=-1)(
        delayed(lambda cluster_label: (cluster_label, xyz[labels == cluster_label]))(
            label
        )
        for label in valid_labels
    )

    clusters = dict(clusters)

    return clusters, labels


class CompensationStrategy(ABC):
    """
    Abstract Base Class (Strategy) for all Lidar compensation methods.

    Defines the interface and common buffering logic for handling PointCloud2 data
    and separating points belonging to the ego vehicle from the environment.
    """

    def __init__(self):
        super().__init__()
        self._cur_lidar_data: Optional[PointCloud2] = None
        self._prev_lidar_data: Optional[PointCloud2] = None

    def set_lidar_data(self, data: PointCloud2):
        """
        Updates the Lidar data buffer, shifting the current data to the previous
        buffer before setting the new data.

        :param data: The incoming Lidar PointCloud2 message.
        """

        if self._cur_lidar_data is not None:
            self._prev_lidar_data = self._cur_lidar_data

        self._cur_lidar_data = data

    def valid_lidar_data(self):
        """
        Checks if both current and previous Lidar data buffers are populated,
        which is required for motion compensation.

        :return: True if both buffers are valid, False otherwise.
        """

        if self._cur_lidar_data is None:
            rclpy.logging.get_logger("lidar_distance").error(
                "Current Lidar data must be set before compensating."
            )
            return False

        if self._prev_lidar_data is None:
            return False

        return True

    def prepare_data(self):
        """
        Converts buffered PointCloud2 messages to NumPy arrays and segments
        the previous frame into ego vehicle points and environment points.
        """

        self.cur_points = ros2_numpy.point_cloud2.pointcloud2_to_array(
            self._cur_lidar_data
        )
        self.prev_points = ros2_numpy.point_cloud2.pointcloud2_to_array(
            self._prev_lidar_data
        )

        ego_mask = create_ego_vehicle_mask(self.prev_points)
        self.prev_ego_points = self.prev_points[ego_mask]
        self.prev_env_points = self.prev_points[~ego_mask]

    @abstractmethod
    def set_motion_data(self, **kwargs):
        """
        Abstract method for receiving specific motion data (e.g., Pose, Velocity).
        Must be implemented by concrete strategies.
        """
        pass

    @abstractmethod
    def compensate(self) -> PointCloud2:
        """
        Abstract method for computing the compensated point cloud.
        Must be implemented by concrete strategies.

        :return: The resulting combined/compensated PointCloud2 message.
        """
        pass


class NoCompensation(CompensationStrategy):
    """
    Strategy: Returns only the current Lidar cloud (no buffering or compensation).
    Acts as a pass-through or baseline mode.
    """

    def set_motion_data(self, **kwargs):
        """No motion data is required for this strategy."""

        pass

    def compensate(self) -> PointCloud2:
        """
        Returns the current Lidar data directly.
        """

        self.valid_lidar_data()  # Ensures the current data is set
        return self._cur_lidar_data


class Buffer(CompensationStrategy):
    """
    Strategy: Returns a simple concatenation of the current and previous Lidar cloud.
    This provides spatial buffering but no motion correction.
    """

    def set_motion_data(self, **kwargs):
        """No motion data is required for this strategy."""

        pass

    def compensate(self) -> PointCloud2:
        """
        Concatenates the current and previous Lidar points.

        :return: A single PointCloud2 containing both frames.
        """

        if not self.valid_lidar_data():
            return self._cur_lidar_data

        self.prepare_data()

        lidar_data = np.concatenate([self.cur_points, self.prev_points])
        lidar_cloud = ros2_numpy.point_cloud2.array_to_pointcloud2(lidar_data)
        lidar_cloud.header = self._cur_lidar_data.header
        return lidar_cloud


class EgoMotionCompensation(CompensationStrategy):
    """
    Strategy: Uses local EKF poses to compute and apply ego motion compensation
    to the environment points from the previous frame.
    """

    def __init__(self):
        super().__init__()
        self._prev_ekf_pose: Optional[PoseStamped] = None
        self._cur_ekf_pose: Optional[PoseStamped] = None

    def valid_ekf_data(self) -> bool:
        """Checks if both current and previous EKF poses are buffered."""

        if self._cur_ekf_pose is None:
            rclpy.logging.get_logger("lidar_distance").error(
                "Current EKF pose must be set before compensating."
            )
            return False

        if self._prev_ekf_pose is None:
            return False

        return True

    def set_motion_data(self, data: Optional[PoseStamped] = None, **kwargs):
        """
        Receives and buffers PoseStamped data (local EKF pose).

        :param data: The incoming PoseStamped message.
        """

        if data is not None:
            if self._cur_ekf_pose is not None:
                self._prev_ekf_pose = self._cur_ekf_pose

            self._cur_ekf_pose = data

    def compensate(self) -> PointCloud2:
        """
        Calculates the delta transformation matrix (dT) from EKF poses and applies
        it to the previous frame's environment points.

        The resulting cloud is
        [Current Frame] + [Compensated Environment] + [Previous Ego].
        """

        if not self.valid_lidar_data() or not self.valid_ekf_data():
            return self._cur_lidar_data

        self.prepare_data()

        dT = create_delta_matrix(self._cur_ekf_pose, self._prev_ekf_pose)
        comp_env_points = ego_motion_compensation(self.prev_env_points, dT)

        lidar_points = np.concatenate(
            [self.cur_points, comp_env_points, self.prev_ego_points]
        )

        lidar_cloud = ros2_numpy.point_cloud2.array_to_pointcloud2(lidar_points)
        lidar_cloud.header = self._cur_lidar_data.header

        return lidar_cloud


class LocalCompensation(CompensationStrategy):
    """
    Strategy: Uses local vehicle dynamics (velocity and heading change) to apply
    a simplified (X-translation and Yaw rotation) compensation.
    """

    def __init__(self):
        super().__init__()

        self._prev_heading: Optional[float] = None
        self._cur_heading: Optional[float] = None
        self.delta_heading: Optional[float] = None
        self._velocity: Optional[float] = None

    def valid_heading_data(self) -> bool:
        """Checks if both current and previous heading data are buffered."""

        if self._cur_heading is None:
            rclpy.logging.get_logger("lidar_distance").error(
                "Current heading must be set before compensating."
            )
            return False

        if self._prev_heading is None:
            return False

        return True

    def valid_velocity_data(self) -> bool:
        """Checks if velocity data is available."""

        if self._velocity is None:
            rclpy.logging.get_logger("lidar_distance").error(
                "Current velocity must be set before compensating."
            )
            return False

        return True

    def set_motion_data(
        self,
        heading: Optional[float] = None,
        velocity: Optional[float] = None,
        **kwargs,
    ):
        """
        Receives and buffers local motion parameters (heading and velocity).
        """

        if heading is not None:
            if self._cur_heading is not None:
                self._prev_heading = self._cur_heading

            self._cur_heading = heading

        if velocity is not None:
            self._velocity = velocity

    def compensate(self) -> PointCloud2:
        """
        Calculates d_x (distance traveled) and d_heading (yaw change) and applies
        the local compensation to the previous frame's environment points.
        """

        if (
            not self.valid_lidar_data()
            or not self.valid_heading_data()
            or not self.valid_velocity_data()
        ):
            return self._cur_lidar_data

        self.prepare_data()

        t_prev = (
            self._prev_lidar_data.header.stamp.sec
            + self._prev_lidar_data.header.stamp.nanosec / 1e9
        )
        t_cur = (
            self._cur_lidar_data.header.stamp.sec
            + self._cur_lidar_data.header.stamp.nanosec / 1e9
        )

        d_t = t_cur - t_prev
        d_x = self._velocity * d_t
        self.delta_heading = self._prev_heading - self._cur_heading

        comp_env_points = apply_local_motion_compensation(
            self.prev_env_points, d_x, self.delta_heading
        )

        lidar_points = np.concatenate(
            [self.cur_points, comp_env_points, self.prev_ego_points]
        )

        lidar_cloud = ros2_numpy.point_cloud2.array_to_pointcloud2(lidar_points)
        lidar_cloud.header = self._cur_lidar_data.header

        return lidar_cloud


def main(args=None):
    rclpy.init(args=args)

    try:
        node = LidarDistance()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

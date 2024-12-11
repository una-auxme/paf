#!/usr/bin/env python
from joblib import Parallel, delayed
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge

# from mpl_toolkits.mplot3d import Axes3D
# from itertools import combinations
# from matplotlib.colors import LinearSegmentedColormap


class LidarDistance:
    """See doc/perception/lidar_distance_utility.md on
    how to configute this node
    """

    def __init__(self):
        self.cluster_buffer = []

    def callback(self, data):
        """
        Callback function that processes LiDAR point cloud data.

        Executes clustering and image calculations for the provided point cloud.

        :param data: LiDAR point cloud as a ROS PointCloud2 message.
        """
        self.start_clustering(data)
        self.start_image_calculation(data)

    def listener(self):
        """
        Initializes the ROS node, creates publishers/subscribers, and keeps it active.
        """
        rospy.init_node("lidar_distance")
        self.bridge = CvBridge()  # OpenCV bridge for image conversions

        # Publisher for filtered point clouds
        self.pub_pointcloud = rospy.Publisher(
            rospy.get_param(
                "~point_cloud_topic",
                "/carla/hero/" + rospy.get_namespace() + "_filtered",
            ),
            PointCloud2,
            queue_size=1,
        )

        # Publishers for distance images in various directions
        self.dist_array_center_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Center/dist_array"),
            ImageMsg,
            queue_size=10,
        )
        self.dist_array_back_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Back/dist_array"),
            ImageMsg,
            queue_size=10,
        )
        self.dist_array_lidar_publisher = rospy.Publisher(
            rospy.get_param(
                "~image_distance_topic_cluster", "/paf/hero/dist_clustered"
            ),
            PointCloud2,
            queue_size=10,
        )
        rospy.loginfo("dist_array_lidar_publisher successfully created.")
        self.dist_array_left_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Left/dist_array"),
            ImageMsg,
            queue_size=10,
        )
        self.dist_array_right_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Right/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        # Subscriber for LiDAR data (point clouds)
        rospy.Subscriber(
            rospy.get_param("~source_topic", "/carla/hero/LIDAR"),
            PointCloud2,
            self.callback,
        )

        rospy.loginfo("Lidar Processor Node started.")
        rospy.spin()

    def start_clustering(self, data):
        """
        Filters LiDAR point clouds, performs clustering,
        and publishes the combined clusters.

        :param data: LiDAR point clouds in ROS PointCloud2 format.
        """

        # Filter point clouds to remove irrelevant data
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        filtered_coordinates = coordinates[
            ~(
                (coordinates["x"] >= -2)
                & (coordinates["x"] <= 2)
                & (coordinates["y"] >= -1)
                & (coordinates["y"] <= 1)
            )  # Exclude points related to the ego vehicle
            & (
                coordinates["z"] > -1.7 + 0.05
            )  # Minimum height in z to exclude the road
        ]

        # Compute cluster data from the filtered coordinates
        clustered_points = cluster_lidar_data_from_pointcloud(
            coordinates=filtered_coordinates
        )

        # Only store valid cluster data
        if clustered_points:
            self.cluster_buffer.append(clustered_points)
        else:
            rospy.logwarn("No cluster data generated.")

        # Combine clusters
        combined_clusters = combine_clusters(self.cluster_buffer)

        self.cluster_buffer = []

        # Publish the combined clusters
        self.publish_clusters(combined_clusters, data.header)

    def publish_clusters(self, combined_clusters, data_header):
        """
        Publishes combined clusters as a ROS PointCloud2 message.

        :param combined_clusters: Combined point clouds of the clusters as a structured
         NumPy array.
        :param data_header: Header information for the ROS message.
        """
        # Convert to a PointCloud2 message
        pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(combined_clusters)
        pointcloud_msg.header = data_header
        pointcloud_msg.header.stamp = rospy.Time.now()
        # Publish the clusters
        self.dist_array_lidar_publisher.publish(pointcloud_msg)

    def start_image_calculation(self, data):
        """
        Computes distance images based on LiDAR data and publishes them.

        :param data: LiDAR point cloud in ROS PointCloud2 format.
        """
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

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
        LiDAR coordinates.

        :param coordinates: Filtered LiDAR coordinates as a NumPy array.
        :param focus: The focus direction ("Center", "Back", "Left", "Right").
        :return: Distance image as a 2D array.
        """
        # Define bounding box parameters based on focus direction
        bounding_box_params = {
            "Center": {"max_x": np.inf, "min_x": 0.0, "min_z": -1.6},
            "Back": {"max_x": 0.0, "min_x": -np.inf, "min_z": -1.6},
            "Left": {"max_y": np.inf, "min_y": 0.0, "min_z": -1.6},
            "Right": {"max_y": -0.0, "min_y": -np.inf, "min_z": -1.6},
        }

        # Select parameters for the given focus
        params = bounding_box_params.get(focus)
        if not params:
            rospy.logwarn(f"Unknown focus: {focus}. Skipping image calculation.")
            return None

        # Apply bounding box filter
        reconstruct_bit_mask = lidar_filter_utility.bounding_box(coordinates, **params)
        reconstruct_coordinates = coordinates[reconstruct_bit_mask]

        # Remove the "intensity" field and convert to a NumPy array
        reconstruct_coordinates_xyz = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates, "intensity"
            ).tolist()
        )

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
        Reconstructs a 2D image from 3D LiDAR data for a given camera perspective.

        :param coordinates_xyz: 3D coordinates of the filtered LiDAR points.
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

        # Process each point in the point cloud
        for c in coordinates_xyz:
            if focus == "Center":  # Compute image for the center view
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(m, point)  # Project 3D point to 2D image coordinates
                x, y = int(pixel[0] / pixel[2]), int(
                    pixel[1] / pixel[2]
                )  # Normalize coordinates
                if (
                    0 <= x <= 1280 and 0 <= y <= 720
                ):  # Check if coordinates are within image bounds
                    img[719 - y][1279 - x] = c[0]  # Set depth value
                    dist_array[719 - y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            if focus == "Back":  # Compute image for the rear view
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if 0 <= x <= 1280 and 0 <= y < 720:
                    img[y][1279 - x] = -c[0]
                    dist_array[y][1279 - x] = np.array(
                        [-c[0], c[1], c[2]], dtype=np.float32
                    )

            if focus == "Left":  # Compute image for the left view
                point = np.array([c[0], c[2], c[1], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if 0 <= x <= 1280 and 0 <= y <= 720:
                    img[719 - y][1279 - x] = c[1]
                    dist_array[y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            if focus == "Right":  # Compute image for the right view
                point = np.array([c[0], c[2], c[1], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if 0 <= x < 1280 and 0 <= y < 720:
                    img[y][1279 - x] = -c[1]
                    dist_array[y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

        return dist_array


def array_to_pointcloud2(points, header="hero/Lidar"):
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
    pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(points_structured)

    # Set the timestamp and header for the message
    pointcloud_msg.header.stamp = rospy.Time.now()
    pointcloud_msg.header = header

    return pointcloud_msg


def combine_clusters(cluster_buffer):
    """
    Combines clusters from multiple point clouds into a structured NumPy array.

    :param cluster_buffer: List of dictionaries containing cluster IDs and point clouds.
    :return: Combined structured NumPy array with fields "x", "y", "z", "cluster_id".
    """
    points_list = []
    cluster_ids_list = []

    for clustered_points in cluster_buffer:
        for cluster_id, points in clustered_points.items():
            if points.size > 0:  # Ignore empty clusters
                points_list.append(points)
                # Create an array with the cluster ID for all points in the cluster
                cluster_ids_list.append(
                    np.full(points.shape[0], cluster_id, dtype=np.float32)
                )

    if not points_list:  # If no points are present
        return np.array(
            [], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cluster_id", "f4")]
        )

    # Combine all points and cluster IDs into two separate arrays
    all_points = np.vstack(points_list)
    all_cluster_ids = np.concatenate(cluster_ids_list)

    # Create a structured array for the combined data
    combined_points = np.zeros(
        all_points.shape[0],
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cluster_id", "f4")],
    )
    combined_points["x"] = all_points[:, 0]
    combined_points["y"] = all_points[:, 1]
    combined_points["z"] = all_points[:, 2]
    combined_points["cluster_id"] = all_cluster_ids

    return combined_points


def cluster_lidar_data_from_pointcloud(coordinates, eps=0.3, min_samples=10):
    """
    Performs clustering on LiDAR data using DBSCAN and returns the clusters.

    :param coordinates: LiDAR point cloud as a NumPy array with "x", "y", "z".
    :param eps: Maximum distance between points to group them into a cluster.
    :param min_samples: Minimum number of points required to form a cluster.
    :return: Dictionary with cluster IDs and their corresponding point clouds.
    """
    if coordinates.shape[0] == 0:
        rospy.logerr("The input array 'coordinates' is empty.")
        return {}

    # Extract x, y, and z from the coordinates for DBSCAN clustering
    xyz = np.column_stack((coordinates["x"], coordinates["y"], coordinates["z"]))

    if xyz.shape[0] == 0:
        rospy.logwarn("No data points available for DBSCAN. Skipping clustering.")
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

    return clusters


if __name__ == "__main__":
    """
    Initialisiert die LidarDistance-Klasse und startet die Listener-Methode.
    """
    lidar_distance = LidarDistance()
    lidar_distance.listener()

#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from std_msgs.msg import String, Header
from sensor_msgs.msg import PointCloud2, PointField
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import json
from sensor_msgs import point_cloud2
from visualization_msgs.msg import Marker, MarkerArray
from tf.transformations import quaternion_from_matrix
from std_msgs.msg import Float32MultiArray
import struct


class RadarNode:
    """See doc/perception/radar_node.md on how to configure this node."""

    def callback(self, data):
        """Process radar Point2Cloud data and publish clustered data points.

        Extracts information from radar data
        and publishes the clustered radar data
        points as a String message.

        Args:
            data: Point2Cloud message containing radar data
        """

        dataarray = pointcloud2_to_array(data)

        self.get_lead_vehicle_info(dataarray)

        # radar position z=0.7
        min_z = float(rospy.get_param("~clustering_radar_z_min", -0.40))
        max_z = float(rospy.get_param("~clustering_radar_z_max", 2.0))
        dataarray = filter_data(dataarray, min_z=min_z, max_z=max_z)

        eps = rospy.get_param("~dbscan_eps", 0.4)
        min_samples = rospy.get_param("~dbscan_min_samples", 3)
        clustered_data = cluster_data(dataarray, eps, min_samples)

        # transformed_data = transform_data_to_2d(dataarray)

        cloud = create_pointcloud2(dataarray, clustered_data.labels_)
        self.visualization_radar_publisher.publish(cloud)

        points_with_labels = np.hstack(
            (dataarray, clustered_data.labels_.reshape(-1, 1))
        )
        bounding_boxes = generate_bounding_boxes(points_with_labels)

        marker_array = MarkerArray()
        for label, bbox in bounding_boxes:
            if label != -1:
                marker = create_bounding_box_marker(label, bbox)
                marker_array.markers.append(marker)
                # can be used for extra debugging
                # min_marker, max_marker = create_min_max_markers(label, bbox)
                # marker_array.markers.append(min_marker)
                # marker_array.markers.append(max_marker)

        self.marker_visualization_radar_publisher.publish(marker_array)

        cluster_info = generate_cluster_info(
            clustered_data, dataarray, marker_array, bounding_boxes
        )
        self.cluster_info_radar_publisher.publish(cluster_info)

    def listener(self):
        """Initializes the node and its publishers."""
        rospy.init_node("radar_node")
        self.dist_array_radar_publisher = rospy.Publisher(
            rospy.get_param(
                "~image_distance_topic", "/paf/hero/Radar/dist_array_unsegmented"
            ),
            String,
            queue_size=10,
        )
        self.visualization_radar_publisher = rospy.Publisher(
            rospy.get_param("~visualisation_topic", "/paf/hero/Radar/Visualization"),
            PointCloud2,
            queue_size=10,
        )
        self.marker_visualization_radar_publisher = rospy.Publisher(
            rospy.get_param("~marker_topic", "/paf/hero/Radar/Marker"),
            MarkerArray,
            queue_size=10,
        )
        self.cluster_info_radar_publisher = rospy.Publisher(
            rospy.get_param("~clusterInfo_topic_topic", "/paf/hero/Radar/ClusterInfo"),
            String,
            queue_size=10,
        )
        self.range_velocity_radar_publisher = rospy.Publisher(
            rospy.get_param(
                "~range_velocity_topic",
                "/paf/hero/Radar/lead_vehicle/range_velocity_array",
            ),
            Float32MultiArray,
            queue_size=10,
        )
        self.lead_vehicle_marker_publisher = rospy.Publisher(
            rospy.get_param(
                "~lead_vehicle_marker_topic", "/paf/hero/Radar/lead_vehicle/marker"
            ),
            Marker,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~source_topic", "/carla/hero/RADAR"),
            PointCloud2,
            self.callback,
        )
        rospy.spin()

    def get_lead_vehicle_info(self, radar_data):
        """
        Processes radar data to identify and publish information about the lead vehicle.

        This function filters radar points to identify the closest point within a
        specified region, representing the lead vehicle. It publishes the distance
        and velocity of the lead vehicle as a `Float32MultiArray` message and also
        visualizes the lead vehicle using a marker in RViz.

        Args:
            radar_data (np.ndarray): Radar data represented as a 2D NumPy array where
                                    each row corresponds to a radar point with the
                                    format [x, y, z, velocity].

        Returns:
            None: The function publishes data to relevant ROS topics and does not return
            any value.
        """

        # radar is positioned at z = 0.7
        radar_data = filter_data(
            radar_data, max_x=20, min_y=-1, max_y=1, min_z=-0.45, max_z=0.8
        )

        lead_vehicle_info = Float32MultiArray()

        # Handle the case where no valid radar points are found
        if len(radar_data) == 0:
            lead_vehicle_info.data = []
            self.range_velocity_radar_publisher.publish(lead_vehicle_info)
            return

        # Identify the closest point (lead vehicle candidate) based on the x-coordinate
        closest_point = radar_data[np.argmin(radar_data[:, 0])]

        lead_vehicle_info.data = [closest_point[0], closest_point[3]]

        # Create a marker for visualizing the lead vehicle in RViz
        marker = Marker()
        marker.header.frame_id = "hero/RADAR"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lead_vehicle_marker"
        marker.id = 500
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = closest_point[0]
        marker.pose.position.y = closest_point[1]
        marker.pose.position.z = closest_point[2]
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.lead_vehicle_marker_publisher.publish(marker)
        self.range_velocity_radar_publisher.publish(lead_vehicle_info)
        return


def pointcloud2_to_array(pointcloud_msg):
    """
    Converts a ROS PointCloud2 message into a NumPy array and calculates the
    Euclidean distance of each point from the origin.

    Parameters:
    - pointcloud_msg: sensor_msgs/PointCloud2
        The ROS PointCloud2 message containing the 3D points.

    Returns:
    - np.ndarray
        A 2D array where each row corresponds to a point in the point cloud:
        [x, y, z, Velocity]
    """
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud_msg)
    return np.column_stack(
        (cloud_array["x"], cloud_array["y"], cloud_array["z"], cloud_array["Velocity"])
    )


def filter_data(
    data,
    min_x=-100,
    max_x=100,
    min_y=-100,
    max_y=100,
    min_z=-1,
    max_z=100,
    max_distance=100,
):
    """
    Filters radar data based on specified spatial and distance constraints.

    This function applies multiple filtering criteria to the input radar data.
    Points outside these bounds are excluded from the output.

    Args:
        data (np.ndarray): A 2D numpy array containing radar data, where each row
        represents a data point with the format [x, y, z, distance]. The array
        shape is (N, 4), where N is the number of points.
        min_x (float, optional): Minimum value for the x-coordinate. Default is -100.
        max_x (float, optional): Maximum value for the x-coordinate. Default is 100.
        min_y (float, optional): Minimum value for the y-coordinate. Default is -100.
        max_y (float, optional): Maximum value for the y-coordinate. Default is 100.
        min_z (float, optional): Minimum value for the z-coordinate. Default is -1.
        max_z (float, optional): Maximum value for the z-coordinate. Default is 100.
        max_distance (float, optional): Maximum allowable distance of the point from
        the sensor. Default is 100.

    Returns:
        np.ndarray: A numpy array containing only the filtered data points that meet
        the specified criteria.
    """

    filtered_data = data[data[:, 3] < max_distance]
    filtered_data = filtered_data[
        (filtered_data[:, 0] >= min_x)
        & (filtered_data[:, 0] <= max_x)
        & (filtered_data[:, 1] >= min_y)
        & (filtered_data[:, 1] <= max_y)
        & (filtered_data[:, 2] <= max_z)
        & (filtered_data[:, 2] >= min_z)
    ]
    return filtered_data


def cluster_data(data, eps, min_samples):
    """
    Clusters the radar data using the DBSCAN algorithm

    Args:
        data (np.ndarray): data array which should be clustered
        eps (float, optional): maximum distance of points. Defaults to 0.8.
        min_samples (int, optional): min samples for 1 cluster. Defaults to 3.

    Returns:
        dict: A dictionary where the keys are cluster labels (int) and the values
              are the number of points in each cluster. Returns an empty dictionary
              if no points are available.
        DBSCAN: A DBSCAN clustering object containing labels and core sample indices
    """

    if len(data) == 0:
        return {}
    scaler = StandardScaler()
    data_scaled = scaler.fit_transform(data)
    clustered_points = DBSCAN(eps=eps, min_samples=min_samples).fit(data_scaled)

    return clustered_points


# generates random color for cluster
def generate_color_map(num_clusters):
    np.random.seed(42)
    colors = np.random.randint(0, 255, size=(num_clusters, 3))
    return colors


def create_pointcloud2(clustered_points, cluster_labels):
    """_summary_

    Args:
        clustered_points (dict): clustered points after dbscan
        cluster_labels (_type_): _description_

    Returns:
        PointCloud2: pointcloud which can be published
    """
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "hero/RADAR"

    points = []
    unique_labels = np.unique(cluster_labels)
    colors = generate_color_map(len(unique_labels))

    for i, point in enumerate(clustered_points):
        x, y, z, v = point
        label = cluster_labels[i]

        if label == -1:
            r, g, b = 128, 128, 128
        else:
            r, g, b = colors[label]

        rgb = struct.unpack("f", struct.pack("I", (r << 16) | (g << 8) | b))[0]
        points.append([x, y, z, rgb])

    fields = [
        PointField("x", 0, PointField.FLOAT32, 1),
        PointField("y", 4, PointField.FLOAT32, 1),
        PointField("z", 8, PointField.FLOAT32, 1),
        PointField("rgb", 12, PointField.FLOAT32, 1),
    ]

    return point_cloud2.create_cloud(header, fields, points)


def transform_data_to_2d(clustered_data):
    """_summary_

    Args:
        clustered_data (np.ndarray): clustered 3d data points

    Returns:
        _np.ndarray: clustered points, every z value is set to 0
    """

    transformed_points = clustered_data
    transformed_points[:, 0] = clustered_data[:, 0]
    transformed_points[:, 1] = clustered_data[:, 1]
    transformed_points[:, 2] = 0
    transformed_points[:, 3] = clustered_data[:, 3]

    return transformed_points


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

    # for 2d (top-down) boxes
    # x_min = np.min(cluster_points[:, 0])
    # x_max = np.max(cluster_points[:, 0])
    # y_min = np.min(cluster_points[:, 1])
    # y_max = np.max(cluster_points[:, 1])
    # rospy.loginfo(f"Bounding box: X({x_min}, {x_max}), Y({y_min}, {y_max})")
    # return x_min, x_max, y_min, y_max

    # for 3d boxes
    x_min = np.min(cluster_points[:, 0])
    x_max = np.max(cluster_points[:, 0])
    y_min = np.min(cluster_points[:, 1])
    y_max = np.max(cluster_points[:, 1])
    z_min = np.min(cluster_points[:, 2])
    z_max = np.max(cluster_points[:, 2])
    return x_min, x_max, y_min, y_max, z_min, z_max


def generate_bounding_boxes(points_with_labels):
    """
    Generates bounding boxes for clustered points.

    This function processes a set of points, each associated with a cluster label,
    and generates an axis-aligned bounding box (AABB) for each unique cluster label.

    Args:
        points_with_labels (numpy.ndarray):
        A 2D array of shape (N, 4) where each row contains
        the coordinates (x, y, z) of a point along with its
        corresponding cluster label in the last column.
        The array should have the structure [x, y, z, label].

    Returns:
        list: A list of tuples, where each tuple contains a cluster label and the
              corresponding bounding box (bbox). The bbox is represented by a tuple
              of the form (x_min, x_max, y_min, y_max, z_min, z_max).
    """
    bounding_boxes = []
    unique_labels = np.unique(points_with_labels[:, -1])
    for label in unique_labels:
        if label == -1:
            continue
        cluster_points = points_with_labels[points_with_labels[:, -1] == label, :3]
        bbox = calculate_aabb(cluster_points)
        bounding_boxes.append((label, bbox))
    return bounding_boxes


def create_bounding_box_marker(label, bbox, bbox_type="aabb"):
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
    marker.header.frame_id = "hero/RADAR"  # Reference frame for the marker
    marker.ns = "marker_radar"  # Namespace to group related markers
    marker.id = int(label)  # Use the label as the unique marker ID
    marker.lifetime = rospy.Duration(0.1)  # Marker visibility duration in seconds
    marker.type = Marker.CUBE  # Use a cube for the bounding box
    marker.action = Marker.ADD  # Action to add or modify the marker

    # Set marker color and opacity
    marker.color.r = 1.0  # Red
    marker.color.g = 0.5  # Green
    marker.color.b = 0.0  # Blue
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
        quaternion = quaternion_from_matrix(np.vstack([eigenvectors.T, [0, 0, 0]]).T)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]

        marker.scale.x = width
        marker.scale.y = length
        marker.scale.z = height

    else:
        raise ValueError(f"Unsupported bbox_type: {bbox_type}")

    return marker


# can be used for extra debugging
def create_min_max_markers(
    label,
    bbox,
    frame_id="hero/RADAR",
    min_color=(0.0, 1.0, 0.0, 1.0),
    max_color=(1.0, 0.0, 0.0, 1.0),
):
    """
    creates RViz-Markers for min- and max-points of a bounding box.

    Args:
        label (int): cluster-id (used as marker-ID in rviz).
        bbox (tuple): min- and max-values of bounding box
        (x_min, x_max, y_min, y_max, z_min, z_max).
        frame_id (str): frame ID for markers
        min_color (tuple): RGBA-value for min-point-marker
        max_color (tuple): RGBA-value for max-point-marker

    Returns:
        tuple: pair of markers (min_marker, max_marker).
    """
    x_min, x_max, y_min, y_max, z_min, z_max = bbox

    # min-point-marker
    min_marker = Marker()
    min_marker.header.frame_id = frame_id
    min_marker.id = int(label * 10)
    min_marker.type = Marker.SPHERE
    min_marker.action = Marker.ADD
    min_marker.scale.x = 0.2
    min_marker.scale.y = 0.2
    min_marker.scale.z = 0.2
    min_marker.color.r = min_color[0]
    min_marker.color.g = min_color[1]
    min_marker.color.b = min_color[2]
    min_marker.color.a = min_color[3]
    min_marker.pose.position.x = x_min
    min_marker.pose.position.y = y_min
    min_marker.pose.position.z = z_min

    # max-point-marker
    max_marker = Marker()
    max_marker.header.frame_id = frame_id
    max_marker.id = int(label * 10 + 1)
    max_marker.type = Marker.SPHERE
    max_marker.action = Marker.ADD
    max_marker.scale.x = 0.2
    max_marker.scale.y = 0.2
    max_marker.scale.z = 0.2
    max_marker.color.r = max_color[0]
    max_marker.color.g = max_color[1]
    max_marker.color.b = max_color[2]
    max_marker.color.a = max_color[3]
    max_marker.pose.position.x = x_max
    max_marker.pose.position.y = y_max
    max_marker.pose.position.z = z_max

    return min_marker, max_marker


# generates string with label-id and cluster size, can be used for extra debugging
def generate_cluster_info(clusters, data, marker_array, bounding_boxes):
    """
    Generates information about clusters, including the label, number of points,
    markers, and bounding boxes.

    Args:
        clusters (DBSCAN): The clustered data, containing the labels for each point.
        data (numpy.ndarray):
        The point cloud data, typically with columns [x, y, z, distance].
        marker_array (MarkerArray):
        The array of RViz markers associated with the clusters.
        bounding_boxes (list): The list of bounding boxes for each detected object.

    Returns:
        str: A JSON string containing the information about each cluster, including:
             - "label": The cluster label.
             - "points_count": The number of points in the cluster.
             - "Anzahl marker": The number of markers in the MarkerArray.
             - "Anzahl Boundingboxen": The number of bounding boxes.
    """
    cluster_info = []

    for label in set(clusters.labels_):
        cluster_points = data[clusters.labels_ == label]
        cluster_size = len(cluster_points)
        if label != -1:
            cluster_info.append(
                {
                    "label": int(label),
                    "points_count": cluster_size,
                    "num_marker": len(marker_array.markers),
                    "num_bounding_boxes": len(bounding_boxes),
                }
            )

    return json.dumps(cluster_info)


if __name__ == "__main__":
    radar_node = RadarNode()
    radar_node.listener()

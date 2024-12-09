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
from geometry_msgs.msg import Point

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
        # clustered_points = cluster_radar_data_from_pointcloud(data, 10)
        # clustered_points_json = json.dumps(clustered_points)
        # self.dist_array_radar_publisher.publish(clustered_points_json)

        # output array [x, y, z, distance]
        dataarray = pointcloud2_to_array(data)

        # input array [x, y, z, distance], max_distance, output: filtered data
        # dataarray = filter_data(dataarray, 10)

        # input array [x, y, z, distance], output: dict clustered
        clustered_data = cluster_data(dataarray)

        # transformed_data = transform_data_to_2d(dataarray)

        # input array [x, y, z, distance], clustered labels
        cloud = create_pointcloud2(dataarray, clustered_data.labels_)
        self.visualization_radar_publisher.publish(cloud)

        points_with_labels = np.hstack((dataarray, clustered_data.labels_.reshape(-1, 1)))
        bounding_boxes = generate_bounding_boxes(points_with_labels)

        marker_array = MarkerArray()
        # marker_array = clear_old_markers(marker_array)
        for label, bbox in bounding_boxes:
            if label != -1:
                marker = create_bounding_box_marker(label, bbox)
                marker_array.markers.append(marker)
                # min_marker, max_marker = create_min_max_markers(label, bbox)
                # marker_array.markers.append(min_marker)
                # marker_array.markers.append(max_marker)

        marker_array = clear_old_markers(marker_array, max_id=len(bounding_boxes) - 1)

        self.marker_visualization_radar_publisher.publish(marker_array)

        cluster_info = generate_cluster_labels_and_colors(clustered_data, dataarray, marker_array, bounding_boxes)
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
            rospy.get_param(
                "~image_distance_topic", "/paf/hero/Radar/Visualization"
            ),
            PointCloud2,
            queue_size=10,
        )
        self.marker_visualization_radar_publisher = rospy.Publisher(
            rospy.get_param(
                "~image_distance_topic", "/paf/hero/Radar/Marker"
            ),
            MarkerArray,
            queue_size=10,
        )
        self.cluster_info_radar_publisher = rospy.Publisher(
            rospy.get_param(
                "~image_distance_topic", "/paf/hero/Radar/ClusterInfo"
            ),
            String,
            queue_size=10,
        )
        rospy.Subscriber(
            rospy.get_param("~source_topic", "/carla/hero/RADAR"),
            PointCloud2,
            self.callback,
        )
        rospy.spin()


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
        [x, y, z, distance], where "distance" is the distance from the origin.
    """
    cloud_array = ros_numpy.point_cloud2.pointcloud2_to_array(pointcloud_msg)
    # distances = np.sqrt(
    #     cloud_array["x"] ** 2 + cloud_array["y"] ** 2 + cloud_array["z"] ** 2
    # )
    # return np.column_stack(
    #     (cloud_array["x"], cloud_array["y"], cloud_array["z"], distances)
    # )
    return np.column_stack(
        (cloud_array["x"], cloud_array["y"], cloud_array["z"], cloud_array["Velocity"])
    )


def cluster_radar_data_from_pointcloud(
    pointcloud_msg, max_distance, eps=1.0, min_samples=2
):
    """
    Filters and clusters points from a ROS PointCloud2 message based on DBSCAN
    clustering.

    Parameters:
    - pointcloud_msg: sensor_msgs/PointCloud2
        The ROS PointCloud2 message containing the 3D points.
    - max_distance: float
        Maximum distance to consider points. Points beyond this distance are
        discarded.
    - eps: float, optional (default: 1.0)
        The maximum distance between two points for them to be considered in
        the same cluster.
    - min_samples: int, optional (default: 2)
        The minimum number of points required to form a cluster.

    Returns:
    - dict
        A dictionary where the keys are cluster labels (int) and the values
        are the number of points in each cluster. Returns an empty dictionary
        if no points are available.
    """
    data = pointcloud2_to_array(pointcloud_msg)
    filtered_data = data[data[:, 3] < max_distance]
    filtered_data = filtered_data[
        (filtered_data[:, 1] >= -1)
        & (filtered_data[:, 1] <= 1)
        & (filtered_data[:, 2] <= 1.3)
        & (filtered_data[:, 2] >= -0.7)
    ]
    if len(filtered_data) == 0:
        return {}
    coordinates = filtered_data[:, :2]
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coordinates)
    labels = clustering.labels_
    clustered_points = {label: list(labels).count(label) for label in set(labels)}
    clustered_points = {int(label): count for label, count in clustered_points.items()}
    return clustered_points


# filters radar data in distance, y, z direction
def filter_data(data, max_distance):

    # filtered_data = data[data[:, 3] < max_distance]
    filtered_data = data
    filtered_data = filtered_data[
        # (filtered_data[:, 1] >= -1)
        # & (filtered_data[:, 1] <= 1)
        # & (filtered_data[:, 2] <= 1.3)
        (filtered_data[:, 2] <= 1.3)
        & (filtered_data[:, 2] >= -0.6)  # -0.7
    ]
    return filtered_data


# clusters data with DBSCAN
def cluster_data(filtered_data, eps=0.8, min_samples=5):

    if len(filtered_data) == 0:
        return {}
    # coordinates = filtered_data[:, :2]
    # clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coordinates)

    # worse than without scaling
    scaler = StandardScaler()
    data_scaled = scaler.fit_transform(filtered_data)
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(data_scaled)

    # clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_data)
    return clustering


# generates random color for cluster
def generate_color_map(num_clusters):
    np.random.seed(42)
    colors = np.random.randint(0, 255, size=(num_clusters, 3))
    return colors


# creates pointcloud2 for publishing clustered radar data
def create_pointcloud2(clustered_points, cluster_labels):
    header = Header()
    header.stamp = rospy.Time.now()
    header.frame_id = "hero/RADAR"

    points = []
    unique_labels = np.unique(cluster_labels)
    colors = generate_color_map(len(unique_labels))

    for i, point in enumerate(clustered_points):
        # x, y, z, _ = point
        x, y, z, v = point
        label = cluster_labels[i]

        if label == -1:
            r, g, b = 128, 128, 128
        else:
            r, g, b = colors[label]

        rgb = struct.unpack('f', struct.pack('I', (r << 16) | (g << 8) | b))[0]
        points.append([x, y, z, rgb])

    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
        PointField('rgb', 12, PointField.FLOAT32, 1),
    ]

    return point_cloud2.create_cloud(header, fields, points)


def transform_data_to_2d(clustered_data):

    transformed_points = clustered_data
    transformed_points[:, 0] = clustered_data[:, 0]
    transformed_points[:, 1] = clustered_data[:, 1]
    transformed_points[:, 2] = 0
    transformed_points[:, 3] = clustered_data[:, 3]

    return transformed_points


def calculate_aabb(cluster_points):
    """_summary_

    Args:
        cluster_points (_type_): _description_

    Returns:
        _type_: _description_
    """

    # for 2d (top-down) boxes
    # x_min = np.min(cluster_points[:, 0])
    # x_max = np.max(cluster_points[:, 0])
    # y_min = np.min(cluster_points[:, 1])
    # y_max = np.max(cluster_points[:, 1])

    # return x_min, x_max, y_min, y_max

    # for 3d boxes
    x_min = np.min(cluster_points[:, 0])
    x_max = np.max(cluster_points[:, 0])
    y_min = np.min(cluster_points[:, 1])
    y_max = np.max(cluster_points[:, 1])
    z_min = np.min(cluster_points[:, 2])
    z_max = np.max(cluster_points[:, 2])
    rospy.loginfo(f"Bounding box for label: X({x_min}, {x_max}), Y({y_min}, {y_max}), Z({z_min}, {z_max})")
    return x_min, x_max, y_min, y_max, z_min, z_max


def generate_bounding_boxes(points_with_labels):
    """_summary_

    Args:
        points_with_labels (_type_): _description_

    Returns:
        _type_: _description_
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


def create_bounding_box_marker(label, bbox):
    """_summary_

    Returns:
        _type_: _description_
    """
    # for 2d (top-down) boxes
    # x_min, x_max, y_min, y_max = bbox

    # for 3d boxes
    x_min, x_max, y_min, y_max, z_min, z_max = bbox

    marker = Marker()
    marker.header.frame_id = "hero/RADAR"
    marker.id = int(label)
    # marker.type = Marker.LINE_STRIP
    marker.type = Marker.LINE_LIST
    marker.action = Marker.ADD
    marker.scale.x = 0.1
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

    # for 2d (top-down) boxes
    # points = [
    #     Point(x_min, y_min, 0),
    #     Point(x_max, y_min, 0),
    #     Point(x_max, y_max, 0),
    #     Point(x_min, y_max, 0),
    #     Point(x_min, y_min, 0),
    # ]
    # marker.points = points

    # for 3d boxes
    points = [
        Point(x_min, y_min, z_min),  # Ecke 0
        Point(x_max, y_min, z_min),  # Ecke 1
        Point(x_max, y_max, z_min),  # Ecke 2
        Point(x_min, y_max, z_min),  # Ecke 3
        Point(x_min, y_min, z_max),  # Ecke 4
        Point(x_max, y_min, z_max),  # Ecke 5
        Point(x_max, y_max, z_max),  # Ecke 6
        Point(x_min, y_max, z_max),  # Ecke 7

        # Point(x_min, y_min, z_min),  # Verbinde z_min zu z_max
        # Point(x_min, y_min, z_max),

        # Point(x_max, y_min, z_min),
        # Point(x_max, y_min, z_max),

        # Point(x_max, y_max, z_min),
        # Point(x_max, y_max, z_max),

        # Point(x_min, y_max, z_min),
        # Point(x_min, y_max, z_max),
    ]
    # marker.points = points
    lines = [
        (0, 1), (1, 2), (2, 3), (3, 0),  # Boden
        (4, 5), (5, 6), (6, 7), (7, 4),  # Deckel
        (0, 4), (1, 5), (2, 6), (3, 7),  # Vertikale Kanten
    ]
    for start, end in lines:
        marker.points.append(points[start])
        marker.points.append(points[end])

    return marker


def create_min_max_markers(label, bbox, frame_id="hero/RADAR", min_color=(0.0, 1.0, 0.0, 1.0), max_color=(1.0, 0.0, 0.0, 1.0)):
    """
    Erstellt RViz-Marker für die Min- und Max-Punkte einer Bounding Box.

    Args:
        label (int): Die ID des Clusters (wird als Marker-ID genutzt).
        bbox (tuple): Min- und Max-Werte der Bounding Box (x_min, x_max, y_min, y_max, z_min, z_max).
        frame_id (str): Frame ID, in dem die Marker gezeichnet werden.
        min_color (tuple): RGBA-Farbwerte für den Min-Punkt-Marker.
        max_color (tuple): RGBA-Farbwerte für den Max-Punkt-Marker.

    Returns:
        tuple: Ein Paar von Markern (min_marker, max_marker).
    """
    x_min, x_max, y_min, y_max, z_min, z_max = bbox

    # marker = Marker()
    # marker.header.frame_id = "hero/RADAR"
    # marker.id = int(label)
    # # marker.type = Marker.LINE_STRIP
    # marker.type = Marker.LINE_LIST
    # marker.action = Marker.ADD
    # marker.scale.x = 0.1
    # marker.color.r = 1.0
    # marker.color.g = 1.0
    # marker.color.b = 0.0
    # marker.color.a = 1.0

    # Min-Punkt-Marker
    min_marker = Marker()
    min_marker.header.frame_id = frame_id
    min_marker.id = int(label * 10)  # ID für Min-Punkt
    min_marker.type = Marker.SPHERE
    min_marker.action = Marker.ADD
    min_marker.scale.x = 0.2  # Größe des Punktes
    min_marker.scale.y = 0.2
    min_marker.scale.z = 0.2
    min_marker.color.r = min_color[0]
    min_marker.color.g = min_color[1]
    min_marker.color.b = min_color[2]
    min_marker.color.a = min_color[3]
    min_marker.pose.position.x = x_min
    min_marker.pose.position.y = y_min
    min_marker.pose.position.z = z_min

    # Max-Punkt-Marker
    max_marker = Marker()
    max_marker.header.frame_id = frame_id
    max_marker.id = int(label * 10 + 1)  # ID für Max-Punkt
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


def clear_old_markers(marker_array, max_id):
    """Löscht alte Marker aus dem MarkerArray."""
    for marker in marker_array.markers:
        if marker.id >= max_id:
            marker.action = Marker.DELETE
    return marker_array


# generates string with label-id and cluster size
def generate_cluster_labels_and_colors(clusters, data, marker_array, bounding_boxes):
    cluster_info = []

    for label in set(clusters.labels_):
        cluster_points = data[clusters.labels_ == label]
        cluster_size = len(cluster_points)
        if label != -1:
            cluster_info.append({
                "label": int(label),
                "points_count": cluster_size,
                "Anzahl marker": len(marker_array.markers),
                "Anzahl Boundingboxen": len(bounding_boxes)
            })

    return json.dumps(cluster_info)


if __name__ == "__main__":
    radar_node = RadarNode()
    radar_node.listener()

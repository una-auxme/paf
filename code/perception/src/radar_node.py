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

        # input array [x, y, z, distance], clustered labels
        cloud = create_pointcloud2(dataarray, clustered_data.labels_)

        self.visualization_radar_publisher.publish(cloud)

        cluster_info = generate_cluster_labels_and_colors(clustered_data, dataarray)
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

    filtered_data = data[data[:, 3] < max_distance]
    filtered_data = filtered_data[
        (filtered_data[:, 1] >= -1)
        & (filtered_data[:, 1] <= 1)
        & (filtered_data[:, 2] <= 1.3)
        & (filtered_data[:, 2] >= -0.7)
    ]
    return filtered_data


# clusters data with DBSCAN
def cluster_data(filtered_data, eps=0.2, min_samples=1):

    if len(filtered_data) == 0:
        return {}
    # coordinates = filtered_data[:, :2]
    # clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(coordinates)

    # worse than without scaling
    # scaler = StandardScaler()
    # data_scaled = scaler.fit_transform(filtered_data)
    # clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(data_scaled)

    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(filtered_data)
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


# generates string with label-id and cluster size
def generate_cluster_labels_and_colors(clusters, data):
    cluster_info = []

    for label in set(clusters.labels_):
        cluster_points = data[clusters.labels_ == label]
        cluster_size = len(cluster_points)
        if label != -1:
            cluster_info.append({
                "label": int(label),
                "points_count": cluster_size
            })

    return json.dumps(cluster_info)


if __name__ == "__main__":
    radar_node = RadarNode()
    radar_node.listener()

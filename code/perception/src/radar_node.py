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

from tf.transformations import quaternion_from_matrix

import struct
import numpy as np
from math import radians, cos, sin
from ros_compatibility.node import CompatibleNode
import sensor_msgs.point_cloud2 as pc2

import struct

from collections import defaultdict

# from threading import Timer
from rosgraph_msgs.msg import Clock


class RadarNode(CompatibleNode):
    """See doc/perception/radar_node.md on how to configure this node."""

    def __init__(self):

        self.dbscan_eps = float(self.get_param("dbscan_eps", "0.4"))
        self.dbscan_samples = int(self.get_param("~bscan_samples", "3"))

        # collect all data from the sensors
        self.sensor_data_buffer = defaultdict(list)
        # Alternative: only one set of data
        self.sensor_data = {
            "RADAR": None,
            "RADAR2": None,
            "RADAR3": None,
        }

        # Sensor-Konfiguration: [X, Y, Z, Roll, Pitch, Jaw]
        self.sensor_config = {
            "RADAR": [2.0, 0.0, 0.7, 0.0, 0.0, 0.0],
            "RADAR2": [2.0, -1.0, 0.7, 0.0, 0.0, 0.0],
            "RADAR3": [2.0, 1.0, 0.7, 0.0, 0.0, 0.0],
        }
        self.transformation_matrix = {}

        for sensor_name, config in self.sensor_config.items():
            # Create transformation matrix
            self.transformation_matrix[sensor_name] = self.get_transformation_matrix(
                *config
            )

        self.timer_interval = 0.1  # 0.1 seconds

        self.now = 0
        self.previous_time = 0
        self.current_time = 0

        self.datacollecting_started = False

    def transform_point_cloud(self, raw_data, point_step, translation, rotation_angle):
        # Winkel in Radianten umwandeln
        yaw = radians(translation[5])  # rotation_angle)
        cos_yaw, sin_yaw = cos(yaw), sin(yaw)

        # Transformation: Translation und Rotation
        def transform_point(x, y, z):
            # Rotation um die Z-Achse (Yaw)
            x_rot = x * cos_yaw - y * sin_yaw
            y_rot = x * sin_yaw + y * cos_yaw
            # Translation
            x_trans = x_rot + translation[0]
            y_trans = y_rot + translation[1]
            z_trans = z + translation[2]
            return x_trans, y_trans, z_trans

        # Dekodieren der Rohdaten
        points = []
        for i in range(0, len(raw_data), point_step):
            point = struct.unpack("ffffffff", raw_data[i : i + point_step])
            x, y, z = point[0], point[1], point[2]
            # Transformiere den Punkt
            x_t, y_t, z_t = transform_point(x, y, z)
            # Ersetze die Werte und füge weitere Felder hinzu
            transformed_point = (x_t, y_t, z_t, *point[3:])
            points.append(transformed_point)

        # Rekodieren der transformierten Punkte
        transformed_raw_data = b"".join(
            struct.pack("ffffffff", *point) for point in points
        )
        return transformed_raw_data

    """- Timerfunktion um process_data in bestimmten Abstand aufzurufen. 
    - Solange werden alle Daten in Buffer gespeichert"""

    def time_check(self, time):

        sec = time.clock.secs
        nsec = time.clock.nsecs
        nsec /= 1000000000

        self.now = sec + nsec

        if self.datacollecting_started is False:
            return

        if self.previous_time == 0:
            self.previous_time = self.now
            return

        if self.now - self.previous_time > 0.15:
            self.process_data()
            self.previous_time = 0
            return

    def callback(self, data, sensor_name):
        """Process radar Point2Cloud data and publish clustered data points.

        Extracts information from radar data
        and publishes the clustered radar data
        points as a String message.

        Args:
            data: Point2Cloud message containing radar data
        """
        self.datacollecting_started = True
        # Speichere Daten in den Puffer
        # sensor_name = rospy.get_param("~point_cloud_topic").split("/")[-1]

        if sensor_name not in self.sensor_data:
            rospy.logwarn(f"Unknown sensor: {sensor_name}")
            return

        # Safe the newest dataset
        self.sensor_data[sensor_name] = data

        # Check if all sensors received data
        # if all(msg is not None for msg in self.sensor_data.values()):
        #    self.process_data(self.sensor_data)

        self.sensor_data_buffer[sensor_name].append(data)

    def process_data(self):  # , datasets):
        rospy.loginfo("Processing sensor data...")
        combined_points = []

        for sensor_name, messages in self.sensor_data_buffer.items():
            for msg in messages:
                rospy.loginfo(f"Processing data from sensor: {sensor_name}")
                points = self.extract_points(msg, sensor_name)
                combined_points.extend(points)
        self.sensor_data_buffer.clear()

        # for sensor_name, msg in datasets.items():
        #    if msg is not None:
        #        rospy.loginfo(f"Processing data from {sensor_name}")
        #        points = self.extract_points(msg, sensor_name)
        #        combined_points.extend(points)

        if not combined_points:
            rospy.logwarn("No points to process!")
            return

        rospy.loginfo(f"Total combined points: {len(combined_points)}")

        combined_points = np.array(combined_points)

        # combined_points[:, 2] = 0

        # Cluster- und Bounding Box-Verarbeitung
        clustered_data = cluster_data(
            combined_points, eps=self.dbscan_eps, min_samples=self.dbscan_samples
        )
        cloud = create_pointcloud2(combined_points, clustered_data.labels_)
        self.visualization_radar_publisher.publish(cloud)

        points_with_labels = np.hstack(
            (combined_points, clustered_data.labels_.reshape(-1, 1))
        )
        bounding_boxes = generate_bounding_boxes(points_with_labels)

        marker_array = MarkerArray()
        for label, bbox in bounding_boxes:
            rospy.loginfo(f"Label: {label}, Bounding Box: {bbox}")
            marker = create_bounding_box_marker(label, bbox)
            marker_array.markers.append(marker)

        rospy.loginfo(f"Publishing {len(marker_array.markers)} markers.")
        self.marker_visualization_radar_publisher.publish(marker_array)

        cluster_info = generate_cluster_info(
            clustered_data, combined_points, marker_array, bounding_boxes
        )
        rospy.loginfo(f"Cluster Info: {cluster_info}")
        self.cluster_info_radar_publisher.publish(cluster_info)

        # Setze die gespeicherten Nachrichten zurück
        self.sensor_data = {key: None for key in self.sensor_data}

    def process_data2(self):
        # Verarbeite die Daten aller Sensoren
        combined_points = []

        for sensor_name, messages in self.sensor_data_buffer.items():
            for msg in messages:
                points = self.extract_points(msg, sensor_name)
                combined_points.extend(points)

        self.sensor_data_buffer.clear()  # Puffer leeren

        combined_points = np.array(combined_points)

        # Weiterverarbeitung (z. B. Transformation, Clustering, Bounding Boxes)
        self.create_bounding_boxes(combined_points)

    def extract_points2(self, msg, sensor_name):
        # Transformiere die Punkte und konvertiere sie in eine gemeinsame Darstellung
        sensor_config = {
            "RADAR": [2.0, 0.0, 0.7, 0.0, 0.0, 0.0],
            "RADAR2": [2.0, 1.0, 0.7, 0.0, 0.0, 5.0],
            "RADAR3": [2.0, -1.0, 0.7, 0.0, 0.0, 355.0],
        }
        translation = sensor_config[sensor_name]
        data_array = pointcloud2_to_array(msg)

        # radar position z=0.7
        dataarray = filter_data(data_array, min_z=-0.5, max_z=2)

        transformation_matrix = self.get_transformation_matrix(*translation)

        transformed_points = np.array(
            [self.transform_point(point, transformation_matrix) for point in dataarray]
        )

        return transformed_points

    def extract_points(self, msg, sensor_name):
        # Sensorkonfiguration um Transformationsmatrix zu erstellen
        # sensor_config = {
        #    "RADAR": [2.0, 0.0, 0.7, 0.0, 0.0, 0.0],
        #    "RADAR2": [2.0, 1.0, 0.7, 0.0, 0.0, 5.0],
        #    "RADAR3": [2.0, -1.0, 0.7, 0.0, 0.0, 355.0],
        # }

        if sensor_name not in self.sensor_config:
            rospy.logwarn(f"Unknown sensor: {sensor_name}")
            return np.array([])

        # translation = sensor_config[sensor_name]
        data_array = pointcloud2_to_array(msg)

        # Filtere Daten basierend auf Höhenbeschränkungen
        filtered_data = filter_data(data_array, min_z=-0, max_z=2)

        if len(filtered_data) == 0:
            rospy.logwarn(f"No valid points for sensor: {sensor_name}")
            return np.array([])

        # Wende die Transformation an
        # transformation_matrix = self.get_transformation_matrix(*translation)
        transformed_points = np.array(
            [
                self.transform_point(point, self.transformation_matrix[sensor_name])
                for point in filtered_data
            ]
        )

        return transformed_points

    def create_bounding_boxes(self, transformed_points):
        # Punkte clustern
        clustered_data = cluster_data(transformed_points)

        cloud = create_pointcloud2(transformed_points, clustered_data.labels_)
        self.visualization_radar_publisher.publish(cloud)

        points_with_labels = np.hstack(
            (transformed_points, clustered_data.labels_.reshape(-1, 1))
        )
        bounding_boxes = generate_bounding_boxes(points_with_labels)

        # Bounding Boxes visualisieren und veröffentlichen
        marker_array = MarkerArray()
        for label, bbox in bounding_boxes:
            rospy.loginfo(f"Label: {label}, Bounding Box: {bbox}")
            if label != -1:
                marker = create_bounding_box_marker(label, bbox)
                marker_array.markers.append(marker)
        self.marker_visualization_radar_publisher.publish(marker_array)

        cluster_info = generate_cluster_info(
            clustered_data, transformed_points, marker_array, bounding_boxes
        )
        self.cluster_info_radar_publisher.publish(cluster_info)

    def callback2(self, data):
        # Sensorpositionen und -orientierungen (Beispielwerte)
        sensor_config = {
            "RADAR": [2.0, 0.0, 0.7, 0.0, 0.0, 0.0],
            "RADAR2": [2.0, 1.0, 0.7, 0.0, 0.0, 5.0],
            "RADAR3": [2.0, -1.0, 0.7, 0.0, 0.0, 355.0],
        }

        # Get sensor name from topic
        sensor_name = rospy.get_param("~point_cloud_topic").split("/")[-1]

        if sensor_name not in sensor_config:
            rospy.logwarn(f"Unknown sensor: {sensor_name}")
            return

        ######### Transform raw data ##########
        #######################################
        """# Beispielwerte
        point_step = 32  # Punktgröße in Bytes
        translation = sensor_config[
            sensor_name
        ]  # [-1.0, 0.0, 0.0]  # Translation (Beispielwerte)
        rotation_angle = -5  # Drehwinkel (Yaw)

        # Rohdaten des PointCloud2-Objekts (z. B. data.data)
        transformed_data = self.transform_point_cloud(
            bytearray(data.data), point_step, translation, rotation_angle
        )"""

        # Ersetzen der Daten
        # data.data = transformed_data

        ##############################################
        ######### Transform raw data end ##########

        # header = data.header

        # dataarray = pointcloud2_to_array(create_pointcloud2_message(data.header, transformed_points))

        # Optional: Extract velocity after transformation
        # velocities = coordinates["Velocity"]

        # Calculate minimum velocity
        # min_velocity = np.min(velocities)
        # self.dist_array_radar_publisher.publish(Float32(min_velocity))

        dataarray = pointcloud2_to_array(data)

        # radar position z=0.7
        dataarray = filter_data(dataarray, min_z=-0.40, max_z=2)

        ######### Transform filter_data ##########
        ##########################################

        # Extract transformation parameters for this sensorda
        x, y, z, roll, pitch, yaw = sensor_config[sensor_name]
        transformation_matrix = self.get_transformation_matrix(
            x, y, z, roll, pitch, yaw
        )

        # Transform the point cloud
        # coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        # Create an array of points with x, y, z
        # points = np.array([coordinates["x"], coordinates["y"], coordinates["z"]]).T

        # Apply transformation to each point
        transformed_points = np.array(
            [self.transform_point(point, transformation_matrix) for point in dataarray]
        )

        ##############################################
        ######### Transform filter_data end ##########

        clustered_data = cluster_data(transformed_points)  # dataarray)

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

    def get_transformation_matrix(self, x, y, z, roll, pitch, yaw):
        # Convert angles to radians
        roll = np.deg2rad(roll)
        pitch = np.deg2rad(pitch)
        yaw = np.deg2rad(yaw)

        # Rotation matrices
        R_x = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )

        R_y = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        R_z = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        )

        R = np.dot(R_z, np.dot(R_y, R_x))

        # Translation vector
        T = np.array([x, y, z])

        # Homogeneous transformation matrix
        transformation_matrix = np.eye(4)
        transformation_matrix[:3, :3] = R
        transformation_matrix[:3, 3] = T

        return transformation_matrix

    def transform_point(self, point, transformation_matrix):

        # Extract x, y, z and ignore velocity for transformation
        x, y, z, velocity = point

        # Convert to homogeneous coordinates
        homogeneous_point = np.array([x, y, z, 1])

        # Apply transformation
        transformed_point = np.dot(transformation_matrix, homogeneous_point)

        # Return the transformed x, y, z along with the original velocity
        return np.array(
            [transformed_point[0], transformed_point[1], transformed_point[2], velocity]
        )

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
        # rospy.Subscriber( rospy.get_param("~source_topic", "/carla/hero/RADAR"), PointCloud2, self.callback,)
        # rospy.Subscriber( rospy.get_param("~source_topic", "/carla/hero/RADAR2"), PointCloud2, self.callback,)
        # rospy.Subscriber( rospy.get_param("~source_topic", "/carla/hero/RADAR3"), PointCloud2, self.callback,)
        rospy.Subscriber(
            "/carla/hero/RADAR", PointCloud2, self.callback, callback_args="RADAR"
        )
        rospy.Subscriber(
            "/carla/hero/RADAR2", PointCloud2, self.callback, callback_args="RADAR2"
        )
        rospy.Subscriber(
            "/carla/hero/RADAR3", PointCloud2, self.callback, callback_args="RADAR3"
        )
        rospy.Subscriber(
            "/clock",  # rospy.get_param("/clock_topic", "/clock"),
            Clock,
            self.time_check,
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


def cluster_data(data, eps=0.5, min_samples=3):
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
    header.frame_id = "hero"  # /RADAR"

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
    Creates an RViz Marker for visualizing a 3D bounding box.

    This function generates a Marker object for RViz to visualize a 3D bounding box
    based on the provided label and bounding box dimensions. The marker is
    represented as a series of lines connecting the corners of the box.

    Args:
        label (int): The unique identifier for the cluster or object to which the
        bounding box belongs. This label is used as the Marker ID.
        bbox (tuple): A tuple containing the min and max coordinates of the bounding box
                      in the format (x_min, x_max, y_min, y_max, z_min, z_max).

    Returns:
        Marker: A Marker object that can be published to RViz to display the
        3D bounding box. The marker is of type LINE_LIST,
        representing the edges of the bounding box.
    """
    # for 2d (top-down) boxes
    # x_min, x_max, y_min, y_max = bbox

    # for 3d boxes
    x_min, x_max, y_min, y_max, z_min, z_max = bbox

    marker = Marker()
    marker.header.frame_id = "hero"  # /RADAR"
    marker.header.stamp = rospy.Time.now()
    marker.ns = "marker_radar"
    marker.id = int(label)
    marker.lifetime = rospy.Duration(0.15)
    # marker.type = Marker.LINE_STRIP  # 2d boxes
    # marker.type = Marker.LINE_LIST  # 3d boxes
    # marker.action = Marker.ADD
    marker.type = Marker.CUBE  # Use a cube for the bounding box
    marker.action = Marker.ADD  # Action to add or modify the marker
    marker.scale.x = 0.1
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 1.0

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

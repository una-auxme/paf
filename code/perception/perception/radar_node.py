from typing import List
import ros2_numpy
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.duration import Duration
import numpy as np
from std_msgs.msg import String, Header
from sensor_msgs.msg import Imu, PointCloud2, PointField
from sensor_msgs_py import point_cloud2
from sklearn.cluster import DBSCAN

from sklearn.preprocessing import StandardScaler
import json
from visualization_msgs.msg import Marker, MarkerArray
from transforms3d.quaternions import mat2quat
from transforms3d.euler import euler2quat

import struct
from collections import defaultdict
from rosgraph_msgs.msg import Clock
from mapping_common.entity import Motion2D
from mapping_common.transform import Vector2
from mapping_interfaces.msg import ClusteredPointsArray
from paf_common.parameters import update_attributes
from rclpy.parameter import Parameter

from collections import deque

from .perception_utils import array_to_clustered_points


class RadarNode(Node):
    """See doc/perception/radar_node.md on how to configure this node."""

    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")
        # collect all data from the sensors
        self.sensor_data_buffer = defaultdict(list)
        # Alternative: only one set of data
        self.sensor_data = {
            "RADAR0": None,
            "RADAR1": None,
        }
        # Sensor-Konfiguration: [X, Y, Z] # , Roll, Pitch, Jaw]
        self.sensor_config = {
            "RADAR0": [2.0, -1.5, 0.5],  # , 0.0, 0.0, 0.0],
            "RADAR1": [2.0, 0.0, 0.5],  # , 0.0, 0.0, 0.0],
        }

        self.timer_interval = 0.1  # 0.1 seconds

        self.now = 0
        self.previous_time = 0
        self.current_time = 0

        self.datacollecting_started = False

        self.accel_x_buffer = deque(maxlen=5)
        self.accel_z_buffer = deque(maxlen=5)

        self.current_pitch = 0.0

        # Parameters

        self.accelerometer_arrow_size = (
            self.declare_parameter(
                "accelerometer_arrow_size",
                2.0,
            )
            .get_parameter_value()
            .double_value
        )
        self.accelerometer_factor = (
            self.declare_parameter(
                "accelerometer_factor",
                0.05,
            )
            .get_parameter_value()
            .double_value
        )
        self.imu_debug = (
            self.declare_parameter(
                "imu_debug",
                False,
            )
            .get_parameter_value()
            .bool_value
        )

        self.dbscan_eps = (
            self.declare_parameter(
                "dbscan_eps",
                0.3,
            )
            .get_parameter_value()
            .double_value
        )
        self.dbscan_samples = (
            self.declare_parameter(
                "dbscan_samples",
                2,
            )
            .get_parameter_value()
            .integer_value
        )
        self.data_buffered = (
            self.declare_parameter(
                "data_buffered",
                False,
            )
            .get_parameter_value()
            .bool_value
        )
        self.data_buffer_time = (
            self.declare_parameter(
                "data_buffer_time",
                0.1,
            )
            .get_parameter_value()
            .double_value
        )
        self.enable_clustering = (
            self.declare_parameter(
                "enable_clustering",
                False,
            )
            .get_parameter_value()
            .bool_value
        )
        self.enable_debug_info = (
            self.declare_parameter(
                "enable_debug_info",
                False,
            )
            .get_parameter_value()
            .bool_value
        )

        # Publishers
        self.visualization_radar_publisher = self.create_publisher(
            PointCloud2, "/paf/hero/Radar/Visualization", 10
        )
        self.marker_visualization_radar_publisher = self.create_publisher(
            MarkerArray, "/paf/hero/Radar/Marker", 10
        )

        self.entity_radar_publisher = self.create_publisher(
            ClusteredPointsArray, "/paf/hero/Radar/clustered_points", 10
        )
        self.cluster_info_radar_publisher = self.create_publisher(
            String, "/paf/hero/Radar/ClusterInfo", 10
        )
        self.visualization_radar_break_filtered = self.create_publisher(
            PointCloud2, "/paf/hero/Radar/radar_break_filtered", 10
        )
        self.combined_points = self.create_publisher(
            PointCloud2, "/paf/hero/Radar/combined_points", 10
        )
        self.marker_pub = self.create_publisher(
            Marker, "/paf/hero/Radar/IMU/ground_filter/debug_marker", 10
        )

        # Subscribers
        self.create_subscription(
            PointCloud2,
            "/carla/hero/RADAR0",
            lambda msg: self.callback(msg, "RADAR0"),
            10,
        )
        self.create_subscription(
            PointCloud2,
            "/carla/hero/RADAR1",
            lambda msg: self.callback(msg, "RADAR1"),
            10,
        )
        self.create_subscription(Clock, "/clock", self.time_check, 10)

        self.create_subscription(Imu, "/carla/hero/IMU", self.imu_callback, 10)

        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def time_check(self, time):
        """
        Checks the current time and triggers data processing
        if the specified buffer time has elapsed.

        Parameters:
        - time: ROS Time
            A ROS time message containing the current time,
            with `secs` and `nsecs` attributes.

        Functionality:
        1. Verifies if data buffering is enabled using the `data_buffered` parameter.
        2. Converts the ROS time to seconds with fractional precision.
        3. Updates the current time (`self.now`).
        4. If data collection has not started, exits the function.
        5. Initializes `self.previous_time` if it is set to 0.
        6. Compares the elapsed time (`self.now - self.previous_time`)
           with the configured buffer time (`self.data_buffer_time`).
        7. If the buffer time is exceeded, calls `process_data` to process sensor data
           and resets `self.previous_time`.

        Returns:
        - None
            The function manages its outputs through internal
            state updates and function calls.
        """

        if not self.data_buffered:
            return

        sec = time.clock.sec
        nsec = time.clock.nanosec / 1e9

        self.now = sec + nsec

        if self.datacollecting_started is False:
            return

        if self.previous_time == 0:
            self.previous_time = self.now
            return

        if self.now - self.previous_time > self.data_buffer_time:  # 0.15:
            self.process_data(self.sensor_data)
            self.previous_time = 0
            return

    def imu_callback(self, msg):
        """Processes IMU messages to calculate the pitch angle and
           update visualization markers.

        Parameters:
        - msg: sensor_msgs/Imu
            Incoming IMU data with linear acceleration.

        Functionality:
        1. Buffers x and z linear acceleration values (last 5 messages).
        2. Computes the pitch angle using the arctangent of the buffered values and
           applies a scaling factor.
        3. Updates a visualization marker with the current pitch state.

        Return:
        - None. The function updates internal states and publishes markers.
        """

        self.accel_x_buffer.append(msg.linear_acceleration.x)
        self.accel_z_buffer.append(msg.linear_acceleration.z)

        # Ensure that we have enough values (min. 5 measuring points)
        if len(self.accel_x_buffer) < 5:
            return

        # Calculate the average value of the last 5 measurements
        accel_x_avg = np.mean(self.accel_x_buffer)
        accel_z_avg = np.mean(self.accel_z_buffer)

        # Calculate pitch angle (in radians)
        self.current_pitch = -np.arctan2(accel_x_avg, accel_z_avg)
        self.current_pitch *= self.accelerometer_factor

        self.current_roll = 0.0

        if self.imu_debug:
            self.publish_ground_projection_marker(self.accelerometer_arrow_size)

    def publish_ground_projection_marker(self, acc_arrow_size):
        """
        Publishes markers to visualize the calculated floor height
        and displays the current pitch angle as text in RViz.
        This function can be activated in the perception.launch file by switching the
        'imu_debug' parameter to 'True'.

        """
        # Arrow marker for the angle of inclination
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "hero"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "ground_filter"
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        arrow_marker.scale.x = acc_arrow_size  # arrowlength
        arrow_marker.scale.y = 0.1  # arrow width
        arrow_marker.scale.z = 0.1  # arrow height

        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        arrow_marker.pose.position.x = 2.0
        arrow_marker.pose.position.y = 0.0
        arrow_marker.pose.position.z = 1.0

        # Orientation of the arrow based on pitch
        quaternion = euler2quat(self.current_roll, self.current_pitch, 0)
        arrow_marker.pose.orientation.x = quaternion[1]
        arrow_marker.pose.orientation.y = quaternion[2]
        arrow_marker.pose.orientation.z = quaternion[3]
        arrow_marker.pose.orientation.w = quaternion[0]

        # Highlighter for the pitch angle
        text_marker = Marker()
        text_marker.header.frame_id = "hero"
        text_marker.header.stamp = self.get_clock().now().to_msg()
        text_marker.ns = "ground_filter"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.scale.z = 0.5  # Textsize
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0

        # Text content: Pitch angle in degrees
        pitch_in_degrees = np.rad2deg(self.current_pitch)
        text_marker.text = f"Pitch: {pitch_in_degrees:.2f}°"

        # Position des Textmarkers (oberhalb des Pfeils)
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 2.0  # show over vehicle

        # Publish the marker
        self.marker_pub.publish(arrow_marker)
        self.marker_pub.publish(text_marker)

    def filter_points(self, points):
        """
        Filters points from a PointCloud2 based on pitch,
        vehicle height and distance.

        Parameters:
        - pointcloud_msg: sensor_msgs/PointCloud2
            The incoming radar points.

        Return:
        - sensor_msgs/PointCloud2
            Filtered radar points for which there is no ground reflection.
        """

        filtered_points = []
        filtered_out_points = []

        # Calculation of threshold values based on pitch
        pitch_rad = self.current_pitch
        pitch_slope = np.tan(pitch_rad)

        if points is None:
            return

        # Filter Mask: Check whether the point is below the calculated floor height
        # plus a safety offset of 30cm
        mask = points[:, 2] > points[:, 0] * pitch_slope - 0.15

        filtered_points = points[mask]
        filtered_out_points = points[~mask]

        # return None if there are no points left
        # else, return filtered points aswell as filtered out points
        if filtered_points.size == 0:
            return None
        else:
            return filtered_points, filtered_out_points

    def callback(self, data: PointCloud2, sensor_name: str):
        """
        Collects radar sensor data and triggers processing.

        This function:
        - Stores radar sensor data either in a buffer or a single register.
        - Triggers `process_data` when all sensors have provided data.
        - Updates a flag for time-checking logic.

        Args:
            data (sensor_msgs/PointCloud2): Radar sensor data.
            sensor_name (str): Identifier for the radar sensor.

        Raises:
            NotImplementedError: If `sensor_name` is unknown.
        """

        # Sets flag for time_check to know when data is available
        self.datacollecting_started = True

        if sensor_name not in self.sensor_data:
            self.get_logger().warn(f"Unknown sensor: {sensor_name}")
            raise NotImplementedError

        # Save data either in buffer or in single register
        if self.data_buffered:
            # Append data to radar sensor in buffer
            self.sensor_data_buffer[sensor_name].append(data)
        else:
            # Safe the newest dataset in single-data register
            self.sensor_data[sensor_name] = data
            # Check if all sensors received data
            if all(msg is not None for msg in self.sensor_data.values()):
                self.process_data(self.sensor_data)

    def process_data(self, datasets):
        """
        Processes sensor data from a buffer or a single dataset, performs clustering,
        and publishes visualization and cluster information.

        Parameters:
        - datasets: dict
            A dictionary where the keys are sensor names (str) and the values are
            ROS PointCloud2 messages (sensor_msgs/PointCloud2) or None.

        Functionality:
        1. Checks if data is buffered or from a single-data register.
        2. Extracts and transforms points using `extract_points`
           for each sensor message.
        3. Combines the points into a single dataset.
        4. Performs clustering using the DBSCAN algorithm, with parameters `dbscan_eps`
           and `dbscan_samples`.
        5. Creates a new PointCloud2 message with cluster labels and publishes it to a
           visualization topic.
        6. Generates bounding boxes for the clustered points and
           publishes them as markers.
        7. Publishes detailed cluster information, including cluster statistics and
           bounding box details.
        8. Resets buffered and single-register sensor data after processing.

        Returns:
        - None
            The function handles all outputs via ROS publishers and logs.
        """

        combined_points = []
        combined_points_filtered_out = []

        if self.data_buffered:
            # Check all data in buffer
            for sensor_name, messages in self.sensor_data_buffer.items():
                for msg in messages:
                    points = self.extract_points(msg, sensor_name)
                    filter_result = self.filter_points(points)
                    if filter_result is None:
                        continue
                    break_filter_data, break_filter_out_data = filter_result
                    if break_filter_data is not None:
                        combined_points.extend(break_filter_data)
                        combined_points_filtered_out.extend(break_filter_out_data)
            self.sensor_data_buffer.clear()
        else:
            # Check all data in single-data registers
            for sensor_name, msg in datasets.items():
                if msg is not None:
                    points = self.extract_points(msg, sensor_name)
                    filter_result = self.filter_points(points)
                    if filter_result is None:
                        continue
                    break_filter_data, break_filter_out_data = filter_result
                    if break_filter_data is not None:
                        combined_points.extend(break_filter_data)
                        combined_points_filtered_out.extend(break_filter_out_data)

        if not combined_points:
            self.entity_radar_publisher.publish(ClusteredPointsArray())
            self.get_logger().warn("No Radarpoints to process!")
            return

        combined_points_filtered_out = np.array(combined_points_filtered_out)
        cloud2 = create_pointcloud2(
            self.get_clock().now(), combined_points_filtered_out, None, True
        )
        self.visualization_radar_break_filtered.publish(cloud2)

        combined_points = np.array(combined_points)

        # Cluster and bounding box processing

        if self.enable_clustering:
            cluster_labels = cluster_data(
                combined_points,
                eps=self.dbscan_eps,
                min_samples=self.dbscan_samples,
            )

            cloud = create_pointcloud2(
                self.get_clock().now(), combined_points, cluster_labels, False
            )
            self.visualization_radar_publisher.publish(cloud)

            points_with_labels = np.hstack(
                (combined_points, cluster_labels.reshape(-1, 1))
            )
            bounding_boxes = generate_bounding_boxes(points_with_labels)

            marker_array = MarkerArray()
            for label, bbox in bounding_boxes:
                marker = create_bounding_box_marker(
                    label, bbox, bbox_lifetime=self.data_buffer_time
                )
                marker_array.markers.append(marker)

            self.marker_visualization_radar_publisher.publish(marker_array)

            header = Header()
            header.stamp = self.get_clock().now().to_msg()
            header.frame_id = "hero/RADAR"

            clusterPointsNpArray = points_with_labels[:, :3]
            indexArray = points_with_labels[:, -1]
            valid_indices = indexArray != -1
            clusterPointsNpArray = clusterPointsNpArray[valid_indices]
            indexArray = indexArray[valid_indices]
            motionArray = calculate_cluster_velocity(points_with_labels)
            motionArray = motionArray[valid_indices]

            motionArray = [m.to_ros_msg() for m in motionArray]

            clusteredpoints = array_to_clustered_points(
                self.get_clock().now(),
                clusterPointsNpArray,
                indexArray,
                motionArray,
                header_id="hero/RADAR",
            )
            self.entity_radar_publisher.publish(clusteredpoints)

            if self.enable_debug_info:
                cluster_info = generate_cluster_info(
                    cluster_labels, combined_points, marker_array, bounding_boxes
                )
                self.cluster_info_radar_publisher.publish(cluster_info)
        else:
            motionArray = [
                Motion2D(Vector2.new(m[3], 0.0)).to_ros_msg() for m in combined_points
            ]
            clusteredpoints = array_to_clustered_points(
                self.get_clock().now(),
                combined_points[:, :3],
                np.arange(len(combined_points)),
                motionArray,
                header_id="hero/RADAR",
            )
            self.entity_radar_publisher.publish(clusteredpoints)

        # Reset the saved messages
        self.sensor_data = {key: None for key in self.sensor_data}

    def extract_points(self, msg, sensor_name):
        """
        Extracts and transforms points from a ROS PointCloud2 message based on
        a specified sensor configuration.

        Parameters:
        - msg: sensor_msgs/PointCloud2
            The ROS PointCloud2 message containing the 3D points.
        - sensor_name: str
            The name of the sensor, used to retrieve its transformation parameters from
            the sensor configuration.

        Returns:
        - np.ndarray
            A 2D array where each row represents a transformed point in the point cloud:
            [x, y, z, velocity]. Returns an empty array
            if the sensor name is not recognized.
        """

        if sensor_name not in self.sensor_config:
            self.get_logger().warn(f"Unknown sensor: {sensor_name}")
            return np.array([])

        data_array = pointcloud2_to_array(msg)

        # Retrieve sensor position in the vehicle coordinate system
        sensor_x, sensor_y, sensor_z = self.sensor_config[sensor_name]

        # TODO: Solve hardcoded inversion of the y-axis
        sensor_y *= -1

        # Apply translation to align the point cloud with the vehicle coordinate system
        translation = np.array([sensor_x, sensor_y, sensor_z])
        transformed_points = np.column_stack(
            (data_array[:, :3] + translation, data_array[:, 3])
        )  # Apply translation to (x, y, z), keep velocity unchanged

        return transformed_points


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

    # Convert PointCloud2 message to a numpy structured array
    cloud_array = ros2_numpy.point_cloud2.pointcloud2_to_array(pointcloud_msg)

    # Stack the x, y, z coordinates with velocity to form a 2D array
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
    Points outside these bounds are excluded.
    (e.g., spatial limits and maximum distance)

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

    # First filter by distance (column index 3 corresponds to distance)
    filtered_data = data[data[:, 3] < max_distance]

    # Apply spatial filtering (x, y, z constraints)
    filtered_data = filtered_data[
        (filtered_data[:, 0] >= min_x)
        & (filtered_data[:, 0] <= max_x)
        & (filtered_data[:, 1] >= min_y)
        & (filtered_data[:, 1] <= max_y)
        & (filtered_data[:, 2] <= max_z)
        & (filtered_data[:, 2] >= min_z)
    ]
    return filtered_data


def cluster_data(data, eps, min_samples) -> np.ndarray:
    """
    Clusters the radar data using the DBSCAN algorithm and returns the cluster labels.

    This function applies DBSCAN clustering to radar data,
    scaling the data first for better clustering performance.
    It returns the cluster labels assigned by DBSCAN, where each
    point is assigned a cluster label, and noise points are labeled as -1.

    Args:
        data (np.ndarray): A 2D array containing radar data. Each row represents a point
                            with the format [x, y, z, velocity], where:
                            - x, y, z are the 3D coordinates of the radar point.
                            - velocity is the velocity associated with that point.
        eps (float): The maximum distance between two samples for them to be considered
                     as in the same neighborhood. Default is 0.8.
        min_samples (int): The number of samples in a neighborhood for a point to be
                           considered as a core point. Default is 3.

    Returns:
        np.ndarray: An array containing the cluster labels assigned by DBSCAN.
                    Points labeled as -1 are considered noise
                    and don't belong to any cluster.

    Notes:
        - If the input data is empty, the function returns an empty array.
        - Data is scaled before clustering for better performance
            using the `StandardScaler`.
        - The function assumes that the input `data` has 4 columns (x, y, z, velocity),
            and the z-values are replaced by 1 for the purpose of clustering.
    """

    if len(data) == 0:
        return np.array([])

    # Scaling the data for better clustering performance
    scaler = StandardScaler()

    # data_reduced = data[:, [0, 1, 3]]
    data_reduced = data
    data_reduced[:, 2] = 1
    data_scaled = scaler.fit_transform(data_reduced)

    # clustered_points = HDBSCAN(min_cluster_size=10).fit(data_scaled)
    clustered_points = DBSCAN(eps=eps, min_samples=min_samples).fit(data_scaled)

    return clustered_points.labels_


def generate_color_map(num_clusters):
    """
    Generates a random RGB color map for a given number of clusters.

    Args:
        num_clusters (int): Number of clusters to generate colors for.

    Returns:
        np.ndarray: An array of shape (num_clusters, 3) containing random RGB colors.

    Notes:
        Uses a fixed seed (42) for reproducibility.
    """
    np.random.seed(42)
    colors = np.random.randint(0, 255, size=(num_clusters, 3))
    return colors


def create_pointcloud2(
    stamp: Time, clustered_points, cluster_labels, filtered_out_points
):
    """
    Creates a PointCloud2 message from radar points with color mapping for clusters.

    Args:
        clustered_points (np.ndarray): 2D array of radar points
            in format [x, y, z, velocity].
        cluster_labels (np.ndarray): Array of cluster labels for each point.
        filtered_out_points (bool): If True, all points are colored red; otherwise,
            clusters are colored based on labels.

    Returns:
        PointCloud2: A PointCloud2 message containing the radar point cloud data.
    """
    header = Header()
    header.stamp = stamp.to_msg()
    header.frame_id = "hero"

    points = []

    # Define colors based on filtering flag
    if not filtered_out_points:
        unique_labels = np.unique(cluster_labels)
        colors = generate_color_map(len(unique_labels))
    else:
        colors = None  # All points will be colored red

    for i, point in enumerate(clustered_points):
        x, y, z, v = point
        # Handle color assignment based on the filtered_out_points flag
        if filtered_out_points:
            r, g, b = 255, 0, 0  # Red color for filtered points
        else:
            label = cluster_labels[i]
            if label == -1:  # Noise points
                r, g, b = 128, 128, 128  # Gray for noise
            else:
                r, g, b = colors[label]  # Assign color from color map

        # Pack the RGB color into a single 32-bit float
        rgb = struct.unpack("f", struct.pack("I", (r << 16) | (g << 8) | b))[0]
        points.append([x, y, z, rgb])

    # Define PointCloud2 fields
    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="rgb", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    return point_cloud2.create_cloud(header, fields, points)


def calculate_aabb(cluster_points):
    """
    Calculates the axis-aligned bounding box (AABB) for a set of 3D points.

    This function computes the minimum and maximum values along each axis (x, y, z)
    for a given set of 3D points, which defines the bounding box that contains
    all points in the cluster.

    Args:
        cluster_points (numpy.ndarray): A 2D array of shape (N, 3) where each row
                                        represents a 3D point (x, y, z).

    Returns:
        tuple: A tuple (x_min, x_max, y_min, y_max, z_min, z_max), representing
               the axis-aligned bounding box (AABB) for the given set of points.
               The values are the minimum and maximum coordinates along the x, y,
               and z axes.
    """
    # Using np.min and np.max on each axis to get the bounding box
    x_min, x_max = np.min(cluster_points[:, 0]), np.max(cluster_points[:, 0])
    y_min, y_max = np.min(cluster_points[:, 1]), np.max(cluster_points[:, 1])
    z_min, z_max = np.min(cluster_points[:, 2]), np.max(cluster_points[:, 2])

    return x_min, x_max, y_min, y_max, z_min, z_max


def generate_bounding_boxes(points_with_labels):
    """
    Generates axis-aligned bounding boxes (AABB) for each unique cluster.

    This function processes a set of points, each associated with a cluster label,
    and computes the bounding box (AABB) for each cluster by finding the minimum
    and maximum coordinates along the x, y, and z axes.

    Args:
        points_with_labels
        (numpy.ndarray): A 2D array of shape (N, 4) where each row contains
                        the coordinates [x, y, z] of a point along with
                        its corresponding cluster label in the last column.

    Returns:
        list: A list of tuples where each tuple contains a cluster label and the
              corresponding bounding box (bbox). The bbox is represented as
              (x_min, x_max, y_min, y_max, z_min, z_max).
    """
    bounding_boxes = []
    unique_labels = np.unique(
        points_with_labels[:, -1]
    )  # Extract unique cluster labels

    for label in unique_labels:
        if label == -1:  # Skip noise points (label -1)
            continue

        # Filter points belonging to the current cluster
        cluster_points = points_with_labels[points_with_labels[:, -1] == label, :3]

        # Calculate the AABB for the cluster points
        bbox = calculate_aabb(cluster_points)

        # Append the label and bounding box to the result list
        bounding_boxes.append((label, bbox))

    return bounding_boxes


def create_bounding_box_marker(label, bbox, bbox_type="aabb", bbox_lifetime=0.1):
    """
    Creates an RViz Marker for visualizing a 3D bounding box (either AABB or OBB).

    Args:
        label (int): Unique identifier for the cluster or object. Used as the Marker ID.
        bbox (tuple): Bounding box information.
                      - For AABB: (x_min, x_max, y_min, y_max, z_min, z_max).
                      - For OBB: (center, dimensions, eigenvectors).
        bbox_type (str): Type of bounding box ("aabb" or "obb").
        bbox_lifetime (float): Duration for which the marker is visible in RViz.

    Returns:
        Marker: A CUBE Marker object that can be published to RViz.
    """
    # Initialize Marker
    marker = Marker()
    marker.header.frame_id = "hero"  # Reference frame for the marker
    marker.ns = "marker_radar"  # Namespace to group related markers
    marker.id = int(label)  # Use the label as the unique marker ID
    marker.lifetime = Duration(
        seconds=bbox_lifetime
    ).to_msg()  # Marker visibility duration
    marker.type = Marker.CUBE  # Use a cube to represent the bounding box
    marker.action = Marker.ADD  # Action to add or modify the marker

    # Set default marker color (semi-transparent yellow)
    marker.color.r, marker.color.g, marker.color.b, marker.color.a = 1.0, 0.5, 0.0, 0.8

    if bbox_type == "aabb":
        # Calculate center and dimensions for AABB
        x_min, x_max, y_min, y_max, z_min, z_max = bbox
        center_x = (x_min + x_max) / 2.0
        center_y = (y_min + y_max) / 2.0
        center_z = (z_min + z_max) / 2.0

        size_x = x_max - x_min
        size_y = y_max - y_min
        size_z = z_max - z_min

        # Set the position and scale for AABB
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = (
            center_x,
            center_y,
            center_z,
        )
        marker.pose.orientation.x = marker.pose.orientation.y = (
            marker.pose.orientation.z
        ) = 0.0
        marker.pose.orientation.w = 1.0  # No rotation for AABB
        marker.scale.x, marker.scale.y, marker.scale.z = size_x, size_y, size_z

    elif bbox_type == "obb":
        # Extract OBB parameters: center, dimensions, eigenvectors
        center, dimensions, eigenvectors = bbox
        width, length, height = dimensions

        # Set the position for OBB
        marker.pose.position.x, marker.pose.position.y, marker.pose.position.z = center

        # Convert eigenvectors (rotation matrix) to quaternion for OBB
        quaternion = mat2quat(np.vstack([eigenvectors.T, [0, 0, 0]]).T)
        (
            marker.pose.orientation.w,
            marker.pose.orientation.x,
            marker.pose.orientation.y,
            marker.pose.orientation.z,
        ) = quaternion

        # Set scale for OBB
        marker.scale.x, marker.scale.y, marker.scale.z = width, length, height

    else:
        raise ValueError(f"Unsupported bbox_type: {bbox_type}")

    return marker


def calculate_cluster_velocity(points_with_labels):
    """
    Computes the average velocity for each labeled cluster and assigns it to each point.

    Parameters:
    - points_with_labels (numpy.ndarray): An array where each row represents a point
      with its corresponding label in the last column. The fourth column (index 3)
      contains the velocity values.

    Returns:
    - numpy.ndarray: An array of Motion2D objects where each entry corresponds to the
      velocity of the cluster the point belongs to. Entries for invalid labels (-1) are
        None.

    Notes:
    - Points with a label of -1 are considered invalid and excluded from velocity
        computation.
    - The output array has the same length as the input array.
    """
    labels = points_with_labels[:, -1]
    valid_mask = labels != -1  # Filter invalid labels
    valid_points = points_with_labels[valid_mask]

    unique_labels = np.unique(valid_points[:, -1])

    # calculate average velocity for each cluster
    avg_velocities = {
        label: np.mean(valid_points[valid_points[:, -1] == label, 3])
        for label in unique_labels
    }

    # Initialize the output array with None and assign velocities for valid points
    motion_array = np.full(len(points_with_labels), None, dtype=object)
    motion_array[valid_mask] = [
        Motion2D(Vector2.new(avg_velocities[label], 0.0), 0.0)
        for label in labels[valid_mask]
    ]

    return motion_array


def generate_cluster_info(cluster_labels, data, marker_array, bounding_boxes):
    """
    Generates information about clusters, including the label, number of points,
    the number of markers, and bounding boxes.
    This function can be activated in the perception.launch file by switching the
    'enable_debug_info' parameter to 'True'.

    Args:
        cluster_labels (numpy.ndarray): The clustered data labels for each point.
        data (numpy.ndarray): The point cloud data,
        typically with columns [x, y, z, distance].
        marker_array (MarkerArray): The array of RViz markers
        associated with the clusters.
        bounding_boxes (list): The list of bounding boxes for each detected object.

    Returns:
        str: A JSON string containing a list of dictionaries with the following keys:
            - "label": The cluster label.
            - "points_count": The number of points in the cluster.
            - "num_marker": The number of markers in the MarkerArray.
            - "num_bounding_boxes": The number of bounding boxes.
    """
    # Get the number of markers and bounding boxes (constant values for all clusters)
    num_markers = len(marker_array.markers)
    num_bounding_boxes = len(bounding_boxes)

    # Use np.unique for efficient label extraction
    unique_labels = np.unique(cluster_labels)

    cluster_info = []

    for label in unique_labels:
        if label != -1:  # Skip the label -1 (invalid points)
            cluster_points = data[cluster_labels == label]
            cluster_size = len(cluster_points)

            cluster_info.append(
                {
                    "label": int(label),
                    "points_count": cluster_size,
                    "num_marker": num_markers,
                    "num_bounding_boxes": num_bounding_boxes,
                }
            )

    return json.dumps(cluster_info)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = RadarNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

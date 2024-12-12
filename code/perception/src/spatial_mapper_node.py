#!/usr/bin/env python3

import math
from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from vision_node_helper import carla_colors
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import (
    Int8MultiArray,
)
from cv_bridge import CvBridge
import numpy as np
import rospy
from sklearn.decomposition import PCA
from shapely.geometry import Polygon
from scipy.spatial import ConvexHull
from sklearn.cluster import DBSCAN

from visualization_msgs.msg import Marker, MarkerArray


class SpatialMapperNode(CompatibleNode):
    """
    SpatialMapperNode:

    The node combines the output of the VisionNode and the LidarDistanceNode
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.center = self.get_param("center")
        self.back = self.get_param("back")
        self.left = self.get_param("left")
        self.right = self.get_param("right")

        self.setup_dist_array_subscription()
        self.setup_marker_publisher()
        self.setup_segmentation_mask_subscriber()

    def setup_dist_array_subscription(self):
        """
        sets up a subscription to the lidar
        depth image of the selected camera angle
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_dist_array,
            topic="/paf/hero/Center/dist_array",
            qos_profile=1,
        )

    def setup_segmentation_mask_subscriber(self):
        """
        sets up a subscriber for segmentation masks
        """

        self.segmentation_mask_subscriber = rospy.Subscriber(
            f"/paf/{self.role_name}/Center/segmentation_masks",
            Int8MultiArray,
            callback=self.segmentation_mask_callback,
        )

    def setup_marker_publisher(self):
        """
        sets up a publisher for visualization markers
        """

        self.marker_publisher = self.new_publisher(
            msg_type=MarkerArray,
            topic=f"/paf/{self.role_name}/visualization_marker_array",
            qos_profile=1,
        )

    def handle_dist_array(self, dist_array):
        """
        This function overwrites the current depth image from
        the lidar distance node with the latest depth image.

        Args:
            dist_array (image msg): Depth image frim Lidar Distance Node
        """
        # callback function for lidar depth image
        # since frequency is lower than image frequency
        # the latest lidar image is saved
        dist_array = self.bridge.imgmsg_to_cv2(
            img_msg=dist_array, desired_encoding="passthrough"
        )
        self.dist_array = dist_array

    def segmentation_mask_callback(self, msg):
        """
        Callback-Funktion zum Empfangen und Verarbeiten der Segmentierungsdaten
        """
        # try:
        # Daten und Layout extrahieren
        flat_data = np.array(msg.data, dtype=np.int8)
        dimensions = [dim.size for dim in msg.layout.dim]

        # Daten zurück in die ursprüngliche Form bringen
        if len(dimensions) == 3:
            segmentation_array = flat_data.reshape(dimensions)
        else:
            rospy.logwarn("Unexpected dimensions in segmentation mask")

        self.process_segmentation_mask(segmentation_array)

        # except Exception as e:
        #    rospy.logerr(f"Error processing segmentation mask: {e}")

    def process_segmentation_mask(self, segmentation_array):
        """
        Verarbeitung der Segmentierungsmaske:
        - Identifiziert Klassen in der Maske
        - Erstellt Marker für jede Klasse
        """
        # MarkerArray zum Sammeln aller Marker
        marker_array = MarkerArray()
        valid_points_counter = len(segmentation_array) + 1

        for i, segmentation_mask in enumerate(segmentation_array):
            # Calculate depth values from self.dist_array
            class_value = np.unique(segmentation_mask)
            class_value = class_value[class_value != 0]
            assert class_value.size == 1
            class_value = class_value[0]
            depth_array = self.calculate_depth_values(segmentation_mask)
            width, length, rotation_angle, center, valid_points = self.extract_car_bbox(
                depth_array
            )
            if (
                width is None
                or length is None
                or rotation_angle is None
                or center is None
            ):
                continue
            # # Linken, Rechten und Nähsten Punkt bekommen
            # left_point, right_point, nearest_point, valid_points = (
            #     self.extract_percentile_points(
            #         depth_array, percent=0.1, depth_threshold=2
            #     )
            # )
            # if left_point is None or right_point is None or nearest_point is None:
            #     continue

            #           # # Marker für Objekt erstellen
            # left_marker = self.get_marker(
            #     point=left_point,
            #     depth=0.1,
            #     width=0.1,
            #     rotation=0,
            #     id=(3 * i),
            #     obj_class=1,
            #     marker_color=(255, 0, 0),
            # )
            # right_marker = self.get_marker(
            #     point=right_point,
            #     depth=0.1,
            #     width=0.1,
            #     rotation=0,
            #     id=(3 * i) + 1,
            #     obj_class=1,
            #     marker_color=(0, 255, 0),
            # )
            # nearest_marker = self.get_marker(
            #     point=nearest_point,
            #     depth=0.1,
            #     width=0.1,
            #     rotation=0,
            #     id=(3 * i) + 2,
            #     obj_class=1,
            #     marker_color=(0, 0, 255),
            # )

            #           # marker_array.markers.append(left_marker)
            # marker_array.markers.append(right_marker)
            # marker_array.markers.append(nearest_marker)

            for point in valid_points:
                valid_points_counter += 1
                marker_array.markers.append(
                    self.get_marker(
                        point=point,
                        depth=0.05,
                        width=0.05,
                        rotation=0,
                        id=valid_points_counter,
                        obj_class=1,
                        marker_color=(255, 255, 255),
                        height=0.05,
                    )
                )

            # object_point, depth, width, rotation = self.calculate_object_marker(
            #    left_point, right_point, nearest_point
            # )

            marker = self.get_marker(
                point=center,
                depth=length,
                width=width,
                rotation=rotation_angle,
                id=i,
                obj_class=class_value,
            )
            marker_array.markers.append(marker)

            # MarkerArray veröffentlichen
        try:
            self.marker_publisher.publish(marker_array)
            rospy.loginfo(f"Published {len(marker_array.markers)} markers.")
        except Exception as e:
            rospy.logerr(f"Error publishing markers: {e}")

    def get_marker(
        self, point, depth, width, rotation, id, obj_class, marker_color=None, height=1
    ):
        if marker_color is None:
            marker_color = carla_colors[obj_class]
        marker = Marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "min_values"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = depth
        marker.scale.y = width
        marker.scale.z = height
        marker.color.a = 1.0
        marker.color.r = marker_color[0]
        marker.color.g = marker_color[1]
        marker.color.b = marker_color[2]
        marker.pose.orientation.z = rotation
        marker.pose.orientation.w = rotation
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 1.70 + point[2]
        marker.lifetime = rospy.Duration(4 / 20)
        return marker

    def run(self):
        self.spin()

    def calculate_depth_values(self, mask):
        """
        Berechnet die Tiefenwerte basierend auf der Maske und den Lidar-Daten
        """
        obj_dist = np.abs(self.dist_array.copy())  # Kopie der Distanzwerte
        output = np.zeros_like(mask, dtype=np.float32)  # Leeres Array in Maskenform
        mask = mask.astype(bool)  # Konvertiere Maske in boolesches Format
        abs_distance = np.sqrt(
            obj_dist[..., 0] ** 2 + obj_dist[..., 1] ** 2 + obj_dist[..., 2] ** 2
        )
        output[mask] = abs_distance[mask]  # Werte nur an Maskenpositionen setzen
        return output

    def extract_percentile_points(self, depth_array, percent=0.2, depth_threshold=1):
        valid_mask = depth_array > 0
        non_zero_mask = (
            ~(self.dist_array[:, :, 0] == 0.0)
            & ~(self.dist_array[:, :, 1] == 0.0)
            & ~(self.dist_array[:, :, 2] == 0.0)
        )  # Prüfe entlang der dritten Dimension
        combined_mask = valid_mask & non_zero_mask
        valid_points = self.dist_array[combined_mask]
        valid_depths = depth_array[combined_mask]
        if len(valid_points) == 0:
            return None, None, None
        min_index = np.argmin(valid_depths)
        closest_point = valid_points[min_index]

        # Get left and right points
        sorted_indices = np.argsort(valid_points[:, 1])  # y-Wert ist die zweite Spalte
        sorted_points = valid_points[sorted_indices]
        sorted_depths = valid_depths[sorted_indices]
        if (percent == 0) or (percent == 1):
            n = 1
        else:
            n = int(np.ceil(len(sorted_points) * percent))
        right_n_points = sorted_points[:n]
        right_n_depths = sorted_depths[:n]
        right_depth_diffs = np.diff(right_n_depths)
        right_significant_change_idx = np.where(right_depth_diffs < -depth_threshold)[0]
        if len(right_significant_change_idx) > 0:
            right_boundary_idx = (
                right_significant_change_idx[-1] + 1
            )  # +1, da Differenz sich auf den nächsten Punkt bezieht
        else:
            right_boundary_idx = 0

        left_n_points = sorted_points[-n:]
        left_n_depths = sorted_depths[-n:]
        left_depth_diffs = np.diff(left_n_depths)

        left_significant_change_idx = np.where(left_depth_diffs > depth_threshold)[0]
        if len(left_significant_change_idx) > 0:
            left_boundary_idx = left_significant_change_idx[
                0
            ]  # -1, da Differenz sich auf den nächsten Punkt bezieht
        else:
            left_boundary_idx = n - 1
        assert left_boundary_idx >= 0 and left_boundary_idx < n
        assert right_boundary_idx >= 0 and right_boundary_idx < n
        return (
            left_n_points[left_boundary_idx],
            right_n_points[right_boundary_idx],
            closest_point,
            valid_points,
        )

    def extract_car_bbox(self, depth_array):
        valid_mask = depth_array > 0
        non_zero_mask = (
            ~(self.dist_array[:, :, 0] == 0.0)
            & ~(self.dist_array[:, :, 1] == 0.0)
            & ~(self.dist_array[:, :, 2] == 0.0)
        )  # Prüfe entlang der dritten Dimension
        combined_mask = valid_mask & non_zero_mask
        valid_points = self.dist_array[combined_mask]
        # Step 0: filter point cloud
        valid_points = self.cluster_filter(valid_points)
        if valid_points is None:
            rospy.logwarn("No valid points found. Returning None.")
            return None, None, None, None, None

        # Step 1: Project points to 2D (XY plane)
        points_2d = valid_points[:, :2]

        # Step 2: Compute the minimum bounding rectangle
        hull_points = ConvexHull(points_2d).vertices
        polygon = Polygon(points_2d[hull_points])
        min_rectangle = polygon.minimum_rotated_rectangle

        # Extract rectangle corner points
        rect_coords = np.array(min_rectangle.exterior.coords)[
            :-1
        ]  # Remove duplicate endpoint

        # Calculate width and length
        edges = np.linalg.norm(rect_coords - np.roll(rect_coords, -1, axis=0), axis=1)
        width, _, length, _ = sorted(
            edges
        )  # Width is shorter side, length is longer side

        # Calculate rectangle orientation
        # vector = rect_coords[1] - rect_coords[0]
        # rectangle_angle = np.arctan2(vector[1], vector[0])

        # Step 3: Use PCA to refine orientation
        pca = PCA(n_components=2)
        pca.fit(points_2d)
        rotation_angle = np.arctan2(pca.components_[0, 1], pca.components_[0, 0])

        # Step 4: Calculate center of the bounding box
        center_2d = rect_coords.mean(axis=0)
        center = (
            center_2d[0],
            center_2d[1],
            valid_points[:, 2].mean(),
        )  # Add average Z for 3D center

        return width, length, rotation_angle, center, valid_points

    def cluster_filter(self, point_cloud, eps=0.4, min_samples=5):
        """
        Filter point cloud using DBSCAN clustering.

        Parameters:
        - point_cloud (numpy.ndarray): Nx3 array of LiDAR points.
        - eps (float): Maximum distance between points in a cluster.
        - min_samples (int): Minimum number of points in a cluster.

        Returns:
        - filtered_points (numpy.ndarray): Points belonging to the largest cluster.
        """
        db = DBSCAN(eps=eps, min_samples=min_samples).fit(point_cloud)
        labels = db.labels_
        unique_labels = np.unique(labels)
        if len(unique_labels) <= 1 and unique_labels[0] == -1:  # Only noise found
            rospy.logwarn("DBSCAN found no valid clusters. Returning empty array.")
            return None
        valid_labels = labels[labels != -1]
        largest_cluster_label = valid_labels.max()
        filtered_points = point_cloud[labels == largest_cluster_label]

        return filtered_points

    def calculate_object_marker(self, left, right, nearest):
        x_l, y_l, _ = left
        x_r, y_r, _ = right
        x_n, y_n, _ = nearest
        # Breite
        width = math.sqrt((x_r - x_l) ** 2 + (y_r - y_l) ** 2)

        # Tiefe
        depth = abs(
            (y_r - y_l) * x_n - (x_r - x_l) * y_n + x_r * y_l - y_r * x_l
        ) / math.sqrt((y_r - y_l) ** 2 + (x_r - x_l) ** 2)

        # Rotation
        # rotation = math.atan2(y_r - y_l, x_r - x_l)
        rotation = 0

        # Mittelpunkt
        midpoint_x = (x_l + x_r) / 2
        midpoint_y = (y_l + y_r) / 2

        return [midpoint_x, midpoint_y, 0], depth, width, rotation


if __name__ == "__main__":
    roscomp.init("SpatialMapperNode")
    node = SpatialMapperNode("SpatialMapperNode")
    node.run()

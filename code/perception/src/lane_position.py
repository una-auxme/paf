#!/usr/bin/env python
# ROS imports
import rospy
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image as ImageMsg
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge

# intermediate Layer imports
from mapping_common.shape import Rectangle
from mapping_common.entity import Lanemarking, Flags
from mapping_common.transform import Transform2D, Vector2
from mapping_common.map import Map
from mapping.msg import Map as MapMsg

# clustering imports
import numpy as np
from sklearn.cluster import DBSCAN
import cv2

# debugging
import time


class lane_position(CompatibleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.bridge = CvBridge()
        self.dist_arrays = []
        # get camera parameters
        self.camera_x = self.get_param("camera_x")
        self.camera_y = self.get_param("camera_y")
        self.camera_z = self.get_param("camera_z")
        self.camera_roll = self.get_param("camera_roll")
        self.camera_yaw = self.get_param("camera_yaw")
        self.camera_pitch = self.get_param("camera_pitch")
        self.camera_width = self.get_param("camera_width")
        self.camera_height = self.get_param("camera_height")
        self.camera_fov = self.get_param("camera_fov")
        self.camera_coordinates = []
        self.center_x = 5
        self.line_length = 15
        self.line_width = 0.5
        self.zmin = -1.7
        self.z_max = -1.6
        self.epsilon = self.get_param(
            "epsilon_clustering"
        )  # epsilon for clustering algorithm
        self.min_samples = self.get_param(
            "min_samples_clustering"
        )  # min samples for clustering
        self.angle_weight = self.get_param(
            "angle_weight"
        )  # weights for confidence calculation
        self.size_weight = self.get_param("size_weight")
        self.std_dev_weight = self.get_param("std_dev_weight")
        self.angle_normalization = self.get_param(
            "angle_normalization"
        )  # max acceptable angle for normalization
        self.size_normalization = self.get_param(
            "size_normalization"
        )  # max acceptable cluster size for normalization
        self.std_dev_normalization = self.get_param(
            "std_dev_normalization"
        )  # max acceptable standard deviation in linearregression for normalization
        self.angle_prediction_threshold = self.get_param("angle_prediction_threshold")

        self.setup_subscriptions()
        self.setup_publishers()

    def run(self):
        self.spin()
        pass

    def setup_subscriptions(self):
        """
        sets up a subscriber to the lanemask
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.lanemask_handler,
            topic="/paf/hero/Center/lane_mask",
            qos_profile=1,
        )

        """
        sets up a subscription to the lidar
        depth image of the selected camera angle
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.distance_array_handler,
            topic="/paf/hero/Center/dist_array",
            qos_profile=1,
        )

    def setup_publishers(self):
        """sets up a publisher for the lane mask
        topic: /Lane/label_image
        """
        self.label_image_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic="/paf/hero/Lane/label_image",
            qos_profile=1,
        )
        """sets up a publisher for the map with lanemarking entities
        topic: /paf/hero/mapping/init_lanemarkings
        """
        self.map_publisher = self.new_publisher(
            msg_type=MapMsg,
            topic=self.get_param(
                "~map_init_topic", "/paf/hero/mapping/init_lanemarkings"
            ),
            qos_profile=1,
        )

    def lanemask_handler(self, ImageMsg):
        """main function that converts the lanemask to lanemarking entities
        and publishes them"""
        lanemask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        stamp = ImageMsg.header.stamp
        self.edge_detection(lanemask)
        positions, angles, confidences = self.process_lanemask(
            lanemask,
        )

        lanemarkings = self.lanemarking_from_coordinates(
            positions, angles, confidences, stamp, predicted=False
        )
        """predicted_lanemarkings = self.predict_lanemarkings(
            positions, angles, confidences, stamp
        )"""

        self.publish_Lanemarkings_map(lanemarkings)

    def edge_detection(self, lanemask):
        image = np.array(lanemask)
        # Definiere einen Kernel für horizontale Linien
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (50, 1)
        )  # Breiter Kernel für horizontale Strukturen

        # Erkenne horizontale Linien in der Maske
        horizontal_lines = cv2.morphologyEx(image, cv2.MORPH_OPEN, kernel)

        # Entferne die horizontalen Linien aus der Lanemaske
        updated_mask = image - horizontal_lines
        ros_image = self.bridge.cv2_to_imgmsg(updated_mask)
        self.label_image_publisher.publish(ros_image)

    def process_lanemask(self, lanemask):

        points = self.mask_lidarpoints(lanemask)
        clusters = self.cluster_points(points)
        positions, angles, deviations = self.get_lanemarking_position_and_angle(
            clusters
        )
        confidences = self.determine_confidences(angles, clusters, deviations)
        return positions, angles, confidences

    """def process_lanemask(self, lanemask):
        """ """processes the lanemask to extract center position, angle and confidence
        for each found lanemarking

        Args:
            lanemask (_type_): lanemaks


        Returns:
            position, angle and condifence of all lanemarkings"""
    """

        clustering_s = time.time()
        clustered_mask, labels = self.cluster_points(lanemask)
        clustering_e = time.time()
        mask_s = time.time()
        lidar_clusters, cluster_sizes = self.mask_lidarpoints(clustered_mask, labels)
        mask_e = time.time()
        label_s = time.time()
        self.publish_label_image(clustered_mask)
        label_e = time.time()
        lanemarking_s = time.time()
        positions, angles, deviations = self.get_lanemarking_position_and_angle(
            lidar_clusters, cluster_sizes
        )
        lanemarking_e = time.time()
        confidence_s = time.time()
        confidences = self.determine_confidences(angles, cluster_sizes, deviations)
        confidence_e = time.time()
        custering_time = clustering_e - clustering_s
        mask_time = mask_e - mask_s
        label_time = label_e - label_s
        lanemarking_time = lanemarking_e - lanemarking_s
        confidence_time = confidence_e - confidence_s

        return positions, angles, confidences"""

    def lanemarking_from_coordinates(
        self, positions, angles, confidences, stamp, predicted=False
    ):
        """creates a lanemarking entity for the position, angle, confidence,
        timespamp and he predicted flag

        Args:
            positions (_type_): x,ycoordinates of the center
            angles (_type_):angle of the lanemarking
            confidences (_type_): confidence
            stamp (_type_): timestamp
            predicted (bool, optional): is predicted or not. Defaults to False.

        Returns:
           entities: Lanemarking entities
        """
        if positions is None:
            return []
        entities = []
        position_indices = self.calc_position_indices(positions)
        for position, position_index, angle, confidence in zip(
            positions, position_indices, angles, confidences
        ):
            x, y = position
            transform = Transform2D.new_rotation_translation(angle, Vector2.new(x, y))
            shape = Rectangle(
                width=self.line_width, length=self.line_length_to_y_axis(y, angle)
            )  # move to ros param
            style = Lanemarking.Style.SOLID
            flags = Flags(is_lanemark=True)
            timestamp = stamp
            lanemarking = Lanemarking(
                style=style,
                confidence=confidence,
                priority=1,
                shape=shape,
                transform=transform,
                timestamp=timestamp,
                flags=flags,
                predicted=predicted,
                position_index=int(position_index),
            )
            entities.append(lanemarking)
        return entities

    def line_length_to_y_axis(self, x, angle):
        """
        Calculates the total length of a line that is symmetric around (x, y)
        and ends at the y-axis.
        Args:
        x: x-coordinate of the midpoint
        angle: angle of the line in degrees
        Returns: total length of the line
        """

        # Convert angle from degrees to radians
        # theta = np.deg2rad(angle)

        # Calculate the total length of the line
        total_length = (2 * abs(x)) / abs(np.cos(angle))
        return total_length

    def calc_position_indices(self, points):
        """gives each lanemarking a unique index, where 1 is the lane next to the car
        on the left and -1 on the right. higher number indicate, that the marking is
        for the next or further lane.

        Args:
            points: arrays with coordinates of the centers of the lanemarkings

        Returns:
            array with labels for the lanemarkings in the same order as the input array
        """
        if len(points) < 1:
            return [0]
        # Extrahieren der y-Koordinaten
        y_coords = np.array(points)[:, 1]

        # Positive und negative y-Koordinaten trennen
        positive_indices = np.where(y_coords > 0)[0]
        negative_indices = np.where(y_coords < 0)[0]

        # Positive und negative Werte nach Abstand zu 0 sortieren
        sorted_positive_indices = positive_indices[
            np.argsort(y_coords[positive_indices])
        ]
        sorted_negative_indices = negative_indices[
            np.argsort(-y_coords[negative_indices])
        ]

        # Indizes initialisieren
        indices = np.zeros(len(y_coords), dtype=int)

        # Positive Indizes zuweisen
        for rank, idx in enumerate(sorted_positive_indices, start=1):
            indices[idx] = rank

        # Negative Indizes zuweisen
        for rank, idx in enumerate(sorted_negative_indices, start=1):
            indices[idx] = -rank
        return indices

    def publish_Lanemarkings_map(self, Lanemarkings):
        """creates and publishes a Map with all found lanemarking Entities

        Args:
            Lanemarkings: all lanemarking entities
        """
        if Lanemarkings is None:
            return

        stamp = rospy.get_rostime()
        map = Map(timestamp=stamp, entities=Lanemarkings)
        msg = map.to_ros_msg()
        self.map_publisher.publish(msg)

    def determine_confidences(self, angles, cluster_dict, linear_deviations):
        """determines a confidence for every found lanemarking.
        confidence score consists of:
            - Median Angle of the lanemarking to the others,
            - cluster size
            - deviation of the linear regression

        Args:
            angles np.array(rad): all angles between lane marking and car
            cluster_sizes np.array(int): how many pixel in the lanemask contribute
            to the lanemarking
            deviations (float): the standard deviations
            of the linear regression algorithm

        Returns:
            array with confidence scores
        """
        confidences = []
        # Use map to get the lengths of the clusters and convert it to a list
        cluster_sizes = list(map(lambda item: len(item[1]), cluster_dict.items()))
        median_angle_deviations = self.get_median_angle(angles)
        # calculate the confidence for each lanemarking
        for angle, cluster_size, deviation in zip(
            median_angle_deviations, cluster_sizes, linear_deviations
        ):
            # normalize all values so the maximum confidence is 1
            normalized_angle = max(
                0, 1 - (abs(angle) / np.deg2rad(self.angle_normalization))
            )
            normalized_size = min(1, cluster_size / self.size_normalization)
            normalized_std_dev = max(0, 1 - (deviation / self.std_dev_normalization))

            confidence = (
                self.angle_weight * normalized_angle
                + self.size_weight * normalized_size
                + self.std_dev_weight * normalized_std_dev
            )
            confidences.append(confidence)

        return confidences

    def get_median_angle(self, angles):
        """calculates the median angle for every angle to the others

        Args:
            angles : array with angles in radiant
        Returns:
            median_angle_deviations: array with median deviations
        """
        angles = np.array(angles)
        # calculate the median deviation to other lanemarkings
        angle_diff = np.abs(angles[:, None] - angles)
        # Set diagonal to np.nan, to ignore it while calculating the median
        np.fill_diagonal(angle_diff, np.nan)
        median_angle_deviations = np.nanmedian(angle_diff, axis=1)
        return median_angle_deviations

    def mask_lidarpoints(self, lanemask):
        points = self.dist_arrays[lanemask == 255]
        points = points[~np.all(points == [0.0, 0.0, 0.0], axis=1)]
        return points

    """def mask_lidarpoints(self, clustered_mask, labels):
        """ """filters out all lidarpoints, that dont collide with the lane mask.

        Args:
            clustered_mask: lanemask divided in clusters for each lanemarking
            labels: array with clustering labels
        Returns:
            lidar_clusters: arrays with all valid lidara points divided by clusters
            cluster_sizes: size of the lanemask clusters in pixels"""
    """
        lidar_clusters = []
        cluster_sizes = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue
            size = np.count_nonzero(clustered_mask == label)
            points_with_label = self.dist_arrays[clustered_mask == label]
            points = points_with_label[
                ~np.all(points_with_label == [0.0, 0.0, 0.0], axis=1)
            ]
            if len(points) != 0:
                lidar_clusters.append(points)
                cluster_sizes.append(size)
        return lidar_clusters, cluster_sizes"""

    def distance_array_handler(self, ImageMsg):
        dist_array = self.bridge.imgmsg_to_cv2(
            img_msg=ImageMsg, desired_encoding="passthrough"
        )
        self.dist_arrays = dist_array

    def get_lanemarking_position_and_angle(self, clusters):
        """Processes clusters and calculates line fitting for each cluster
        to extract lane markings."""
        lanemarkings = []
        angles = []
        deviations = []

        # Iterate through each cluster in the dictionary
        for label, cluster in clusters.items():
            y = cluster[:, 1]  # y-coordinates
            x = cluster[:, 0]  # x-coordinates

            # Skip clusters with fewer than 3 points
            if len(x) < 3:
                continue

            # Fit a line (linear regression) to the points in the cluster
            p, res = np.polyfit(x, y, deg=1, cov=True)

            # Calculate the standard deviation of the fit
            if np.ndim(res) == 2:
                std_dev = np.sqrt(
                    np.diag(res)
                )  # Standard deviation from the covariance matrix
            else:
                std_dev = np.sqrt(
                    res
                )  # If covariance is a vector, take the square root

            deviations.append(std_dev[0])  # Standard deviation of the slope

            # Get the slope (m) of the line
            m = p[0]

            # Calculate the center of the cluster (mean of x and y coordinates)
            center_y = np.mean(y)
            center_x = np.mean(x)

            # Calculate the angle of the line relative to the x-axis (in radians)
            theta = np.arctan(m)

            # Store the results for lanemarkings, angles, and deviations
            angles.append(theta)
            lanemarkings.append([center_x, center_y])

        return lanemarkings, angles, deviations

    """def get_lanemarking_position_and_angle(self, clusters):
        """ """calculates position and angle of lanemarkings using linear regression
          on given lidar points
        inputs:
            -clusters: array with x and y coordinates of lidar points in clusters
            -cluster_size: size of cluster in lanemask in pixels
            (!important: does not refer to actual number of lidar points
              used for linear regression)
        outputs:
            -lanemarkings: x_center and y_center of all lanemarkings
            -angles: angles of all lanemarkings to the car heading
            -deviations: standard deviations from polyfit (linear regression)"""
    """
        # Berechnung der position und des winkels für jeden Cluster
        lanemarkings = []
        angles = []
        deviations = []
        for cluster in clusters:
            y = cluster[:, 1]
            x = cluster[:, 0]
            if len(x) < 3:
                continue
            p, res = np.polyfit(x, y, deg=1, cov=True)
            if np.ndim(res) == 2:
                std_dev = np.sqrt(np.diag(res))
            else:
                std_dev = np.sqrt(res)
            deviations.append(std_dev[0])

            # Steigung der Geraden
            m = p[0]  # Steigung
            center_y = np.mean(y)
            center_x = np.mean(x)
            # center_x = self.center_x
            theta = np.arctan(m)
            angles.append(theta)

            lanemarkings.append([center_x, center_y])
        return lanemarkings, angles, deviations"""

    def cluster_points(self, points):

        labels = []
        try:
            clustering = DBSCAN(
                eps=self.epsilon, min_samples=self.min_samples, algorithm="ball_tree"
            ).fit(
                points
            )  # HDBSCAN
            labels = clustering.labels_
        except Exception as e:
            print(f"could not cluster given points: {str(e)}")

        # Find the unique cluster labels (excluding -1 for outliers)
        unique_labels = np.unique(labels)

        # Dictionary to store clusters
        clusters = {}

        # Group points by their labels
        for label in unique_labels:
            # Extract all points that belong to the current label
            cluster_points = points[labels == label]

            # If the cluster is not the outlier cluster (-1), store it
            if label != -1:
                clusters[label] = cluster_points
            else:
                clusters["outliers"] = cluster_points  # Store outliers separately

        return clusters

    """ def cluster_points(self, lanemask):
        """ """clusters the lanemask to seperate the different lanemarkings

        Args:
            lanemask: lanemask given by Lanedetection node

        Returns:
            clustered_mask: arrays with size of lanemask filled with labels
            of the clustering
            labels: arrary with the labels for all pixels"""
    """
        labels = []
        clustering = []
        points_s = time.time()
        points = np.argwhere(lanemask == 255)
        clustered_mask = np.zeros_like(lanemask, dtype=int)
        points_e = time.time()
        try:
            # DBSCAN-Clustering
            cluster_s = time.time()
            clustering = DBSCAN(
                eps=self.epsilon, min_samples=self.min_samples, algorithm="ball_tree"
            ).fit(points)
            cluster_e = time.time()
            labels = clustering.labels_ + 1
            mask_s = time.time()
            clustered_mask[points[:, 0], points[:, 1]] = labels
            mask_e = time.time()
            points_time = points_e - points_s
            cluster_time = cluster_e - cluster_s
            mask_time = mask_e - mask_s
        except Exception as e:
            print(f"could not cluster given points: {str(e)}")
        return clustered_mask, labels"""

    def predict_lanemarkings(self, centers, angles, confidences, stamp):
        """predicts new lanemarkings if a lanemarking angle is too steep
        or a lanemarking is found close to the car,
        but the according marking on the other side was not detected

        Args:
            centers (_type_): array with all lanemarking centers
            angles (_type_): array with all lanemarking angles
            confidences (_type_): array with all lanemarking confidences
            stamp (_type_): timestamp of recieved lanemask

        Returns:
            predicted_lanemarkings: predicted lanemarking entities
        """
        new_centers = []
        new_angles = []
        new_confidences = []
        x_coords, y_coords = np.array(centers).T
        closest_lane_index = np.argmin(np.abs(y_coords))
        median_angle_deviations = self.get_median_angle(angles)
        for center, angle, angle_deviation, confidence in zip(
            centers, angles, median_angle_deviations, confidences
        ):
            x, y = center
            if np.abs(angle_deviation) > self.angle_prediction_threshold:
                new_centers.append(center)
                new_angles.append(angle - angle_deviation)
                new_confidences.append(confidence)
            if y == y_coords[closest_lane_index]:
                matching_indices = np.where(
                    (y_coords >= -y - 0.5) & (y_coords <= -y + 0.5)
                )[0]
                if len(matching_indices) == 0:
                    new_centers.append([x, -y])
                    new_angles.append(angle)
                    new_confidences.append(confidence)

        predicted_lanemarkings = self.lanemarking_from_coordinates(
            new_centers, new_angles, new_confidences, stamp, predicted=True
        )
        return predicted_lanemarkings

    def label_to_rgb(self, clustered_mask):
        """converts lanemask to rgb image, where each color is a cluster"""
        # Identifiziere alle einzigartigen Labels im Array
        # (außer 0, falls es als Hintergrund betrachtet wird)
        unique_labels = np.unique(clustered_mask)
        # Entferne alle 0-Werte
        # unique_labels = unique_labels[unique_labels != 0]
        # Beispiel-Dictionary mit festen Farben
        label_to_color = {
            0: [0, 0, 0],  # schwarz
            1: [255, 0, 0],  # rot
            2: [0, 255, 0],  # grün
            3: [0, 0, 255],  # blau
            4: [255, 255, 0],  # gelb
            5: [255, 0, 255],  # magenta
            6: [0, 255, 255],  # cyan
            7: [128, 0, 0],  # dunkelrot
            8: [0, 128, 0],  # dunkelgrün
            9: [0, 0, 128],  # dunkelblau
            10: [128, 128, 0],  # olivgrün
            11: [128, 0, 128],  # lila
            12: [0, 128, 128],  # türkis
            13: [192, 192, 192],  # silber
            14: [255, 165, 0],  # orange
            15: [255, 20, 147],  # deep pink
            16: [34, 139, 34],  # forest green
            17: [255, 105, 180],  # hot pink
            18: [255, 215, 0],  # gold
            19: [0, 191, 255],  # deep sky blue
        }
        # Erstelle ein leeres RGB-Bild
        rgb_image = np.zeros(
            (clustered_mask.shape[0], clustered_mask.shape[1], 3), dtype=np.uint8
        )

        # Fülle das RGB-Bild mit den zugehörigen Farben für jedes Label
        for label, color in label_to_color.items():
            rgb_image[clustered_mask == label] = color

        return rgb_image

    def publish_label_image(self, clustered_mask):
        image = self.label_to_rgb(clustered_mask)
        ros_image = self.bridge.cv2_to_imgmsg(image)
        self.label_image_publisher.publish(ros_image)


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = lane_position("Lanedetection_node")
    node.run()

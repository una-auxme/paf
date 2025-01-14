#!/usr/bin/env python
# ROS imports
import rospy
import cv2
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image as ImageMsg
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge

# intermediate Layer imports
from mapping_common.shape import Rectangle, Circle
from mapping_common.entity import Entity, Lanemarking, Flags
from mapping_common.transform import Transform2D, Vector2
from mapping_common.map import Map
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# from scipy.spatial.transform import Rotation as R
# clustering imports
import numpy as np
from sklearn.cluster import DBSCAN
from scipy.interpolate import griddata
import random


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
        self.epsilon_camera = 0.4
        self.min_samples_camera = 180
        self.epsilon_lidar = 12
        self.min_samples_lidar = 1
        self.confidence_treshold = 1500
        # self.P, self.K, self.R, self.T = self.create_Matrices()

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
        sets up a subscriber to the driveable area
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.driveable_area_handler,
            topic="/paf/hero/Center/driveable_area",
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
        self.coordinate_image_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic="/paf/hero/Center/coordinate_image",
            qos_profile=1,
        )
        self.marker_visualization_camera_publisher = rospy.Publisher(
            rospy.get_param("~marker_topic", "/paf/hero/Lane/Marker_camera"),
            MarkerArray,
            queue_size=10,
        )
        self.marker_visualization_lidar_publisher = rospy.Publisher(
            rospy.get_param("~marker_topic", "/paf/hero/Lane/Marker_lidar"),
            MarkerArray,
            queue_size=10,
        )

        """sets up a publisher for the lane mask
        topic: /Lane/label_image
        """
        self.label_image_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic="/paf/hero/Lane/label_image",
            qos_profile=1,
        )

    def lanemask_handler(self, ImageMsg):
        lanemask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        stamp = ImageMsg.header.stamp
        """marker_array_camera, boundingboxes_camera = self.process_lanemask_camera(
            lanemask,
            red=1.0,
            green=0.5,
            blue=0.5,
            epsilon=self.epsilon_lidar,
            min_samples=self.min_samples_lidar,
        )"""
        marker_array_lidar, boundingboxes_lidar, angles_lidar, confidences = (
            self.process_lanemask_lidar(
                lanemask,
                red=0.5,
                green=1.0,
                blue=0.5,
                epsilon=self.epsilon_lidar,
                min_samples=self.min_samples_lidar,
            )
        )

        # Publish the MarkerArray for visualization
        self.marker_visualization_lidar_publisher.publish(marker_array_lidar)
        # self.marker_visualization_camera_publisher.publish(marker_array_camera)
        Lanemarkings = self.Lanemarking_from_lidar(
            boundingboxes_lidar, angles_lidar, confidences, stamp
        )
        self.publish_Lanemarkings_map(Lanemarkings)

    def Lanemarking_from_lidar(
        self, boundingboxes: np.array, angles: np.array, confidences, stamp
    ):
        if boundingboxes is None:
            return []
        entities = []
        for boundingbox, angle, confidence in zip(boundingboxes, angles, confidences):
            x, y = self.calc_center(boundingbox)
            transform = Transform2D.new_translation(Vector2.new(x, y))
            transform.new_rotation(angle)
            shape = Rectangle(width=0.3, length=15)
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
            )
            entities.append(lanemarking)
        return entities

    def publish_Lanemarkings_map(self, Lanemarkings):
        # Make sure we have data for each dataset we are subscribed to
        if Lanemarkings is None:
            return

        stamp = rospy.get_rostime()
        map = Map(timestamp=stamp, entities=Lanemarkings)
        msg = map.to_ros_msg()
        self.map_publisher.publish(msg)

    def calc_center(self, boundingbox):
        x_center = (
            (boundingbox[0, 0] + boundingbox[1, 0]) / 2
            + (boundingbox[2, 0] + boundingbox[3, 0]) / 2
        ) / 2
        y_center = (
            (boundingbox[0, 1] + boundingbox[1, 1]) / 2
            + (boundingbox[2, 1] + boundingbox[3, 1]) / 2
        ) / 2
        return x_center, y_center

    def process_lanemask_lidar(self, lanemask, red, green, blue, epsilon, min_samples):
        clustered_mask, labels = self.cluster_points(lanemask, epsilon, min_samples)
        lidar_clusters, cluster_confidences = self.mask_lidarpoints(
            clustered_mask, labels
        )
        self.publish_label_image(clustered_mask)
        bounding_boxes, angles = self.get_bounding_boxes(
            lidar_clusters, cluster_confidences
        )
        marker_array = self.create_marker_array(bounding_boxes, red, green, blue)
        return marker_array, bounding_boxes, angles, cluster_confidences

    def process_lanemask_camera(self, lanemask, red, green, blue, epsilon, min_samples):
        clustered_mask, labels = self.cluster_points(lanemask, epsilon, min_samples)
        camera_clusters, cluster_confidences = self.mask_camerapoints(
            clustered_mask, labels
        )
        self.publish_label_image(clustered_mask)
        bounding_boxes = self.get_bounding_boxes(camera_clusters, cluster_confidences)
        marker_array = self.create_marker_array(bounding_boxes, red, green, blue)
        return marker_array, bounding_boxes

    def create_marker_array(self, bounding_boxes, red, green, blue):
        # Create a MarkerArray for visualization
        marker_array = MarkerArray()
        for label, bounding_box in enumerate(bounding_boxes):
            marker = self.create_rectangle_marker(
                label, bounding_box, "marker_camera", red, green, blue
            )
            marker_array.markers.append(marker)
        return marker_array

    def mask_lidarpoints(self, clustered_mask, labels):
        lidar_clusters = []
        cluster_confidences = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue
            confidence = np.count_nonzero(clustered_mask == label)
            points_with_label = self.dist_arrays[clustered_mask == label]
            points = points_with_label[
                ~np.all(points_with_label == [0.0, 0.0, 0.0], axis=1)
            ]
            if len(points) != 0:
                lidar_clusters.append(points)
                cluster_confidences.append(confidence)
        return lidar_clusters, cluster_confidences

    def mask_camerapoints(self, clustered_mask, labels):
        camera_clusters = []
        cluster_confidences = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:
                continue
            confidence = np.count_nonzero(clustered_mask == label)
            points_with_label = self.camera_coordinates[clustered_mask == label]
            points = points_with_label[
                ~np.all(points_with_label == [0.0, 0.0, 0.0], axis=1)
            ]
            if len(points) != 0:
                camera_clusters.append(points)
                cluster_confidences.append(confidence)
        return camera_clusters, cluster_confidences

    def create_rectangle_marker(
        self, label, rectangle, namespace, red=1.0, green=0.5, blue=0.5
    ):
        """
        Creates an RViz Marker for visualizing a 3D bounding box.

        This function generates a Marker object for RViz to visualize a 3D bounding box
        as a series of connected lines representing the edges of the box.

        Args:
            label (int): Unique identifier for the cluster or object.
                        Used as the Marker ID.
            bbox (tuple): Bounding box dimensions in the format:
                        (x_min, x_max, y_min, y_max, z_min, z_max).

        Returns:
            Marker: A LINE_LIST Marker object that can be published to RViz.
        """

        z_min = -1.7
        z_max = -1.2

        # Initialize the Marker object
        marker = Marker()
        marker.header.frame_id = "hero/LIDAR"  # Reference frame for the marker
        marker.ns = "marker_lidar"  # Namespace to group related markers
        marker.id = int(label)  # Use the label as the unique marker ID
        marker.lifetime = rospy.Duration(0.1)  # Marker visibility duration in seconds
        marker.type = Marker.LINE_LIST  # Marker type to represent bounding box edges
        marker.action = Marker.ADD  # Action to add or modify the marker

        # Set marker properties
        marker.scale.x = 0.1  # Line thickness
        marker.color.r = red  # Red color component
        marker.color.g = green  # Green color component
        marker.color.b = blue  # Blue color component
        marker.color.a = 1.0  # Opacity (1.0 = fully visible)

        # Define the 8 corners of the 3D bounding box
        points = [
            Point(rectangle[0][0], rectangle[0][1], z_min),  # Bottom face
            Point(rectangle[1][0], rectangle[1][1], z_min),
            Point(rectangle[2][0], rectangle[2][1], z_min),
            Point(rectangle[3][0], rectangle[3][1], z_min),
            Point(rectangle[0][0], rectangle[0][1], z_max),  # Bottom face
            Point(rectangle[1][0], rectangle[1][1], z_max),
            Point(rectangle[2][0], rectangle[2][1], z_max),
            Point(rectangle[3][0], rectangle[3][1], z_max),
        ]

        # Define lines connecting the corners of the bounding box
        lines = [
            (0, 1),
            (1, 2),
            (2, 3),
            (3, 0),  # Bottom face
            (4, 5),
            (5, 6),
            (6, 7),
            (7, 4),  # Top face
            (0, 4),
            (1, 5),
            (2, 6),
            (3, 7),  # Vertical edges
        ]

        # Add points for each line segment to the marker
        for start, end in lines:
            marker.points.append(points[start])
            marker.points.append(points[end])

        return marker

    def driveable_area_handler(self, ImageMsg):
        """mask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        points = self.get_point_by_angle(mask)
        labels = []
        bounding_boxes = self.get_bounding_boxes(labels, points)"""

    def distance_array_handler(self, ImageMsg):
        dist_array = self.bridge.imgmsg_to_cv2(
            img_msg=ImageMsg, desired_encoding="passthrough"
        )
        self.dist_arrays = dist_array
        if len(self.camera_coordinates) == 0:
            self.camera_coordinates = self.calibrate_camera_coordinates(dist_array)

    def get_bounding_boxes(self, clusters, confidences):
        """
        Berechnet die Bounding Boxes für geclusterte Punkte.

        :param x_coords: x-Koordinaten der Punkte
        :param y_coords: y-Koordinaten der Punkte
        :param epsilon: Maximaler Abstand für DBSCAN-Clustering
        :param min_samples: Minimale Punktanzahl pro Cluster
        :return: Liste von Bounding Boxes [(x_min, y_min, x_max, y_max), ...]
        """
        # Berechnung der Bounding Boxes für jeden Cluster
        bounding_boxes = []
        poly_list = []
        angles = []
        for index, cluster in enumerate(clusters):
            # je gröer das cluster, desto wahrscheinlicher ist es, dass es tatsächlich eine lane markierung ist
            if confidences[index] < self.confidence_treshold:
                continue
            y = cluster[:, 1]
            x = cluster[:, 0]
            poly = np.poly1d(np.polyfit(x, y, deg=1))
            poly_list.append(poly)

            # Steigung und Achsenabschnitt der Geraden
            m = poly.coefficients[0]  # Steigung
            c = poly.coefficients[1]  # Achsenabschnitt
            center_y = np.mean(y)
            theta = np.arctan(m)
            angles.append(theta)
            dx = (self.line_length / 2) * np.cos(theta)
            dy = (self.line_length / 2) * np.sin(theta)
            x1, y1 = self.center_x - dx, center_y - dy
            x2, y2 = self.center_x + dx, center_y + dy

            # Punkte des Rechtecks basierend auf der Breite
            dx_perp = (self.line_width / 2) * np.sin(theta)
            dy_perp = (self.line_width / 2) * np.cos(theta)

            rect_points = [
                (x1 - dx_perp, y1 + dy_perp),
                (x1 + dx_perp, y1 - dy_perp),
                (x2 + dx_perp, y2 - dy_perp),
                (x2 - dx_perp, y2 + dy_perp),
            ]

            bounding_boxes.append(rect_points)
        return bounding_boxes, angles

    def cluster_points(self, lanemask, epsilon, min_samples):
        labels = []
        clustering = []
        points = np.argwhere(lanemask == 255)
        clustered_mask = np.zeros_like(lanemask, dtype=int)
        try:
            # DBSCAN-Clustering
            clustering = DBSCAN(eps=epsilon, min_samples=min_samples).fit(points)
            labels = clustering.labels_ + 1
            clustered_mask[points[:, 0], points[:, 1]] = (
                labels  # +1, um Cluster-ID von 0 anzuheben
            )

        except Exception as e:
            self.get_logger().error(f"could not cluster given points: {str(e)}")
        return clustered_mask, labels

    def get_point_by_angle(self, lanemask):  # nur einmal berrechen
        x_ground = []
        y_ground = []
        try:
            w = self.camera_width
            h = self.camera_height
            theta_h = np.deg2rad(self.camera_fov)  # Horizontaler FOV in Radiant
            theta_v = theta_h * (h / w)  # Vertikaler FOV in Radiant
            # Schritt 1: Pixelgitter erstellen
            u, v = np.meshgrid(np.arange(w), np.arange(h))

            # Schritt 2: Richtungsvektoren berechnen
            d_x = np.ones_like(u, dtype=float)  # x-Richtung bleibt konstant = 1
            d_y = -(u - w / 2) / (w / 2) * np.tan(theta_h / 2)
            d_z = (h / 2 - v) / (h / 2) * np.tan(theta_v / 2)
            d_x = np.where(lanemask, d_x, np.nan)
            d_y = np.where(lanemask, d_y, np.nan)
            d_z = np.where(lanemask, d_z, np.nan)
            # Schritt 3: Parameter t berechnen (z = 0)
            t = -self.camera_z / d_z
            # Schritt 4: Bodenpunkte berechnen
            x_ground = t * d_x
            y_ground = t * d_y
            # Schritt 5: Maske anwenden
            x_ground = x_ground[~np.isnan(x_ground)]
            y_ground = y_ground[~np.isnan(y_ground)]
        except Exception as e:
            self.get_logger().error(f"Failed to calculate distance of Pixel: {str(e)}")

        points = np.stack((x_ground, y_ground), axis=1)
        return points

    def calibrate_camera_coordinates(self, distarray):
        # Extrahiere bekannte Positionen und Werte
        known_positions = np.array(
            np.nonzero(distarray[:, :, 0])
        ).T  # Indizes von Nicht-Null-Punkten
        known_values = distarray[
            known_positions[:, 0], known_positions[:, 1]
        ]  # (x, y, z)-Werte
        # Filtere die bekannten Positionen basierend auf dem z-Wert nahe 0.0
        z_values = known_values[:, 2]
        valid_positions = np.abs(z_values) < 0.25  # Nur Positionen mit |z| < 0.2
        known_positions = known_positions[valid_positions]
        known_values = known_values[valid_positions]
        # Erstelle ein Gitter für die Interpolation
        grid_x, grid_y = np.meshgrid(
            np.arange(distarray.shape[1]),  # Breite
            np.arange(distarray.shape[0]),  # Höhe
        )

        # Interpolation durchführen (für x und y separat, z bleibt niedrig priorisiert)
        interpolated = np.zeros_like(distarray)  # Ergebnisarray (x, y, z)
        for i in range(2):  # Nur x und y interpolieren
            interpolated[:, :, i] = griddata(
                known_positions,  # Stützpunkte
                known_values[:, i],  # Werte für die Dimension (x oder y)
                (grid_y, grid_x),  # Zielgitter (y, x)
                method="linear",  # Interpolationsmethode
                fill_value=np.nan,  # Auffüllen leerer Werte mit NaN
            )

        # Extrapolation für Ränder
        nan_mask = np.isnan(interpolated)
        for i in range(2):  # Nur x und y extrapolieren
            interpolated[:, :, i] = np.where(
                nan_mask[:, :, i],
                griddata(
                    known_positions,
                    known_values[:, i],
                    (grid_y, grid_x),
                    method="nearest",
                ),
                interpolated[:, :, i],
            )

        # Z-Werte direkt aus den bekannten Punkten übernehmen, ohne Interpolation
        interpolated[:, :, 2] = griddata(
            known_positions,
            known_values[:, 2],
            (grid_y, grid_x),
            method="nearest",  # Z-Werte werden einfach übernommen
            fill_value=0.0,
        )
        return interpolated

    def get_points_by_lidar(self, lanemask):
        mask = lanemask != 0
        points = self.dist_arrays[mask]
        points = points[~np.all(points == [0.0, 0.0, 0.0], axis=1)]
        return points

    def project_2d_into_3d(self, lanemask):
        z = self.filter_lanemask(lanemask)
        coordinates = self.filter_image(lanemask)

        K = self.create_K()
        R = self.create_rotation_matrix()
        x = (coordinates[:][0] - K[0][2]) / K[0][0]
        y = (coordinates[:][1] - K[1][2]) / K[1][1]

        X_c = x * z
        Y_c = y * z
        Z_c = z
        C = np.array([[X_c], [Y_c], [Z_c]])
        T = np.array([[self.camera_x], [self.camera_y], [self.camera_z]])
        W = R @ C + T
        return W

    def create_K(self):
        fov_horizontal = np.deg2rad(self.camera_fov)
        fov_vertical = 2 * np.arctan(
            (self.camera_height / self.camera_width) * np.tan(fov_horizontal / 2)
        )
        f_x = self.camera_width / (2 * np.tan(fov_horizontal / 2))
        f_y = self.camera_height / (2 * np.tan(fov_vertical / 2))
        c_x = self.camera_width
        c_y = self.camera_height
        K = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])
        return K

    def create_rotation_matrix(self):
        roll = np.deg2rad(self.camera_roll)
        pitch = np.deg2rad(self.camera_pitch)
        yaw = np.deg2rad(self.camera_yaw)

        R_x = np.array(
            [
                [1, 0, 0],
                [0, np.cos(roll), -np.sin(roll)],
                [0, np.sin(roll), np.cos(roll)],
            ]
        )

        # Rotation matrix around y-axis (Pitch)
        R_y = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        # Rotation matrix around z-axis (Yaw)
        R_z = np.array(
            [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        )

        # Combined rotation matrix (Yaw -> Pitch -> Roll)
        R = np.dot(R_z, np.dot(R_y, R_x))

        return R

    def filter_image(self, lanemask):
        y_coords, x_coords = np.where(lanemask != 0)
        # Combine the x and y coordinates into pairs
        coordinates = np.stack((x_coords, y_coords))
        # coordinates = list(zip(x_coords, y_coords))

        return coordinates

    def filter_lanemask(self, lanemask):
        mask = lanemask != 0
        points = self.dist_arrays[mask]
        points = points[~np.all(points == [0.0, 0.0, 0.0], axis=1)]
        return points

    def get_boundingbox_by_lidar(self, lanemask):
        points = self.filter_lanemask(lanemask)
        # DBSCAN-Clustering
        clustering = DBSCAN(eps=1.0, min_samples=5).fit(points)
        labels = clustering.labels_

        # Berechnung der Bounding Boxes für jeden Cluster
        bounding_boxes = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:  # Rauschen ignorieren
                continue
            cluster_points = points[labels == label]
            x_min, y_min, z_min = cluster_points.min(axis=0)
            x_max, y_max, z_max = cluster_points.max(axis=0)
            bounding_boxes.append((x_min, y_min, x_max, y_max))

        return bounding_boxes

    def generate_random_color(self):
        """Generiert eine zufällige RGB-Farbe."""
        return [random.randint(0, 255) for _ in range(3)]

    def label_to_rgb(self, clustered_mask):
        """Wandelt das Label-Array in ein RGB-Bild um, wobei jedem Label eine zufällige Farbe zugeordnet wird."""
        # Identifiziere alle einzigartigen Labels im Array (außer 0, falls es als Hintergrund betrachtet wird)
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

        """# Erstelle eine Zufallsfarbe für jedes Label
        label_to_color = {
            label: self.generate_random_color() for label in unique_labels
        }"""

        # Fülle das RGB-Bild mit den zugehörigen Farben für jedes Label
        for label, color in label_to_color.items():
            rgb_image[clustered_mask == label] = color

        return rgb_image

    def publish_label_image(self, clustered_mask):
        image = self.label_to_rgb(clustered_mask)
        ros_image = self.bridge.cv2_to_imgmsg(image)
        self.label_image_publisher.publish(ros_image)

    """    def create_coordinate_image(self, x_coords, y_coords):
        # 1. Schritt: Bildgröße bestimmen (maximale Werte aus x und y)
        x_coords = x_coords * 10
        y_coords = y_coords * 10
        image_size = (
            int(np.max(x_coords)) + 10,
            int(np.max(y_coords)) + 10,
        )  # 10 Pixel extra für Rand

        # 2. Schritt: Leeres Bild erstellen
        image = np.zeros(image_size)

        # 3. Schritt: Vektorisierte Berechnung der Pixelkoordinaten
        # Stelle sicher, dass x und y innerhalb der Bildgröße liegen
        x_pixel = np.clip(x_coords.astype(int), 0, image_size[1] - 1)
        y_pixel = np.clip(y_coords.astype(int), 0, image_size[0] - 1)

        # 4. Schritt: Setze alle relevanten Pixel auf 1 (oder eine andere Farbe)
        image[y_pixel, x_pixel] = (
            255  # Setzt die Pixel, die den Koordinaten entsprechen, auf 1 (weiß)
        )
        return image"""

    """def create_Matrices(self):
        f = self.camera_width / (2.0 * np.tan(self.camera_fov * np.pi / 360))
        cx = self.camera_width / 2.0
        cy = self.camera_height / 2.0
        # Interne Kameramatrix
        K = np.array([[f, 0, cx], [0, f, cy], [0, 0, 1]], dtype=np.float64)
        R = self.create_rotation_matrix(
            self.camera_roll, self.camera_pitch, self.camera_yaw
        )
        T = -R @ [self.camera_x, self.camera_y, self.camera_z]
        # Create a grid of pixel coordinates (u, v)
        self.u, self.v = np.meshgrid(
            np.arange(self.camera_width), np.arange(self.camera_height)
        )
        flip = np.array([[0, 1, 0], [0, 0, -1], [1, 0, 0]], dtype=np.float32)

        c_y = np.cos(np.radians(self.camera_yaw))
        s_y = np.sin(np.radians(self.camera_yaw))
        c_r = np.cos(np.radians(self.camera_roll))
        s_r = np.sin(np.radians(self.camera_roll))
        c_p = np.cos(np.radians(self.camera_pitch))
        s_p = np.sin(np.radians(self.camera_pitch))
        matrix = np.identity(4)
        matrix[0, 3] = self.camera_x
        matrix[1, 3] = self.camera_y
        matrix[2, 3] = self.camera_z
        matrix[0, 0] = c_p * c_y
        matrix[0, 1] = c_y * s_p * s_r - s_y * c_r
        matrix[0, 2] = -c_y * s_p * c_r - s_y * s_r
        matrix[1, 0] = s_y * c_p
        matrix[1, 1] = s_y * s_p * s_r + c_y * c_r
        matrix[1, 2] = -s_y * s_p * c_r + c_y * s_r
        matrix[2, 0] = s_p
        matrix[2, 1] = -c_p * s_r
        matrix[2, 2] = c_p * c_r
        matrix = np.linalg.inv(matrix)
        P = K @ flip @ matrix[:3, :]

        return P, K, R, T

    def transform_camera2worldcoordinates(self, lanemask):

        # Flatten the mask and pixel coordinates
        mask_flat = lanemask.flatten()
        u_flat = self.u.flatten()
        v_flat = self.v.flatten()
        # Select only the masked pixels
        masked_indices = np.where(mask_flat == 255)[0]
        u_selected = u_flat[masked_indices]
        v_selected = v_flat[masked_indices]
        # Construct pixel coordinates in homogeneous form
        pixel_coords = np.stack(
            [
                u_selected,
                v_selected,
                np.zeros_like(u_selected),
                np.ones_like(u_selected),
            ],
            axis=0,
        )
        # Transform pixel coordinates to normalized camera coordinates
        norm_camera_coords = self.K_inv @ pixel_coords
        # Project to the ground plane (z_world = 0)
        z_c = self.T[2]  # Camera height
        camera_coords = norm_camera_coords * z_c

        # Transform to world coordinates
        world_coords_homogeneous = np.dot(
            self.R.T, camera_coords - self.T[:, np.newaxis]
        )

        # Extract X, Y world coordinates
        world_coords = world_coords_homogeneous[:2, :].T
        return world_coords

    def create_rotation_matrix(self, roll, pitch, yaw):
        """ """
        Computes the rotation matrix from roll, pitch, and yaw angles.
        :param roll: Rotation around the x-axis (in degrees)
        :param pitch: Rotation around the y-axis (in degrees)
        :param yaw: Rotation around the z-axis (in degrees)
        :return: 3x3 rotation matrix
        """ """
        # Convert angles from degrees to radians
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        # Rotation numpy transposematrix for rotation around the x-axis (Roll)
        R_z = np.array(
            [
                [1, 0, 0],
                [0, np.cos(yaw), -np.sin(yaw)],
                [0, np.sin(yaw), np.cos(yaw)],
            ]
        )

        # Rotation matrix for rotation around the y-axis (Pitch)
        R_y = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        # Rotation matrix for rotation around the z-axis (Yaw)
        R_x = np.array(
            [
                [np.cos(roll), -np.sin(roll), 0],
                [np.sin(roll), np.cos(roll), 0],
                [0, 0, 1],
            ]
        )

        # Combined rotation matrix (R = Rz * Ry * Rx)
        R = R_z @ R_y @ R_x
        return R"""


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = lane_position("Lanedetection_node")
    node.run()

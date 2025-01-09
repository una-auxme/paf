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
from mapping_common.entity import Entity, Lanemarking
from mapping_common.transform import Transform2D, Vector2
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

# from scipy.spatial.transform import Rotation as R
# clustering imports
import numpy as np
from sklearn.cluster import DBSCAN


class lane_position(CompatibleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.bridge = CvBridge()
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
        self.center_x = 5
        self.line_length = 15
        self.line_width = 0.5
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

    def lanemask_handler(self, ImageMsg):
        lanemask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        x_coords, y_coords = self.get_point_by_angle(lanemask)
        bounding_boxes_camera = self.get_bounding_boxes(x_coords, y_coords)
        bounding_boxes_lidar = self.get_bounding_boxes(lanemask)

        # Create a MarkerArray for visualization
        marker_array_camera = MarkerArray()
        for label, bounding_box in enumerate(bounding_boxes_camera):
            marker = self.create_rectangle_marker(
                label, bounding_box, "marker_camera", 0.5, 1.0, 0.5
            )
            marker_array_camera.markers.append(marker)
        # Create a MarkerArray for visualization
        marker_array_lidar = MarkerArray()
        for label, bounding_box in enumerate(bounding_boxes_lidar):
            marker = self.create_bounding_box_marker(
                label, bounding_box, "marker_lidar"
            )
            marker_array_camera.markers.append(marker)

        # Publish the MarkerArray for visualization
        self.marker_visualization_camera_publisher.publish(marker_array_camera)
        # self.marker_visualization_lidar_publisher.publish(marker_array_lidar)

        """transform = Transform2D.new_translation(Vector2.new())
        Lanemarking(
            Lanemarking.Style.SOLID,
        )

        def entities_from_lidar(self) -> List[Entity]:
        if self.lidar_data is None:
            return []

        data = self.lidar_data
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        shape = Circle(0.15)
        lidar_entities = []
        for x, y, z, intensity in coordinates:
            if z < -1.5 or z > 1.0:
                # Ignore street level lidar points and stuff above
                continue
            if random.random() < 0.9:
                # Get rid of points because performance
                continue
            v = Vector2.new(x, y)
            transform = Transform2D.new_translation(v)
            flags = Flags(is_collider=True)
            e = Entity(
                confidence=0.5 * intensity,
                priority=0.25,
                shape=shape,
                transform=transform,
                timestamp=data.header.stamp,
                flags=flags,
            )
            lidar_entities.append(e)

        return lidar_entities
        pass
"""

    def create_bounding_box_marker(
        self, label, bounding_boxes, namespace, red=1.0, green=0.5, blue=0.5
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
        x_min = bounding_boxes[0]
        y_min = bounding_boxes[1]
        x_max = bounding_boxes[2]
        y_max = bounding_boxes[3]

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
            Point(x_min, y_min, z_min),  # Bottom face
            Point(x_max, y_min, z_min),
            Point(x_max, y_max, z_min),
            Point(x_min, y_max, z_min),
            Point(x_min, y_min, z_max),  # Top face
            Point(x_max, y_min, z_max),
            Point(x_max, y_max, z_max),
            Point(x_min, y_max, z_max),
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
        mask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        x_coords, y_coords = self.get_point_by_angle(mask)
        bounding_boxes = self.get_bounding_boxes(x_coords, y_coords)

    def distance_array_handler(self, ImageMsg):
        dist_array = self.bridge.imgmsg_to_cv2(
            img_msg=ImageMsg, desired_encoding="passthrough"
        )
        self.dist_arrays = dist_array

    def get_bounding_boxes(self, x_coords, y_coords, epsilon=0.5, min_samples=180):
        """
        Berechnet die Bounding Boxes für geclusterte Punkte.

        :param x_coords: x-Koordinaten der Punkte
        :param y_coords: y-Koordinaten der Punkte
        :param epsilon: Maximaler Abstand für DBSCAN-Clustering
        :param min_samples: Minimale Punktanzahl pro Cluster
        :return: Liste von Bounding Boxes [(x_min, y_min, x_max, y_max), ...]
        """
        # Stack x und y in ein Array für Clustering
        points = np.stack((x_coords, y_coords), axis=1)
        labels = []
        try:
            # DBSCAN-Clustering
            clustering = DBSCAN(eps=epsilon, min_samples=min_samples).fit(points)
            labels = clustering.labels_

        except Exception as e:
            self.get_logger().error(f"could not cluster given points: {str(e)}")
        # Berechnung der Bounding Boxes für jeden Cluster
        bounding_boxes = []
        unique_labels = set(labels)
        poly_list = []
        for label in unique_labels:
            if label == -1:  # Rauschen ignorieren
                continue
            cluster_points = points[labels == label]

            y = cluster_points[:, 1]
            x = cluster_points[:, 0]
            poly = np.poly1d(np.polyfit(x, y, deg=1))
            poly_list.append(poly)

            # Steigung und Achsenabschnitt der Geraden
            m = poly.coefficients[0]  # Steigung
            c = poly.coefficients[1]  # Achsenabschnitt
            center_y = np.mean(y)
            theta = np.arctan(m)
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

            """x_min, y_min = cluster_points.min(axis=0)
            x_max, y_max = cluster_points.max(axis=0)"""
            bounding_boxes.append(rect_points)
        """image = np.zeros((1280, 720))
        line_image = self.draw_lines(image, poly_list)"""
        return bounding_boxes

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
        return x_ground, y_ground

    def get_boundingbox_by_lidar(self, lanemask):
        mask = lanemask != 0
        points = self.dist_arrays[mask]
        points = points[~np.all(points == [0.0, 0.0, 0.0], axis=1)]
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

    def draw_lines(self, img, lines, color=[255, 0, 0], thickness=3):
        line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
        img = np.copy(img)

        if lines is None:
            return img
        for line in lines:
            for x1, y1, x2, y2 in line:
                cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
        img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
        return img, line_img

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

        # Rotation matrix for rotation around the x-axis (Roll)
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

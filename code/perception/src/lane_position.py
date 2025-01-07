#!/usr/bin/env python
# ROS imports
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image as ImageMsg
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from mapping_common.shape import Rectangle, Circle
from mapping_common.entity import Entity, Lanemarking
from mapping_common.transform import Transform2D, Vector2

# from scipy.spatial.transform import Rotation as R
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
        self.P, self.K, self.R, self.T = self.create_Matrices()

        self.setup_lanemask_subscriptions()
        self.setup_coordinate_image_publisher()

    def run(self):
        self.spin()
        pass

    def setup_lanemask_subscriptions(self):
        """
        sets up a subscriber to the lanemask
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.lanemask_handler,
            topic="/paf/hero/Center/lane_mask",
            qos_profile=1,
        )

    def setup_driveable_area_subscriptions(self):
        """
        sets up a subscriber to the driveable area
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.driveable_area_handler,
            topic="/paf/hero/Center/driveable_area",
            qos_profile=1,
        )

    def setup_coordinate_image_publisher(self):
        self.coordinate_image_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic="/paf/hero/Center/coordinate_image",
            qos_profile=1,
        )

    def lanemask_handler(self, ImageMsg):
        lanemask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        x_coords, y_coords = self.get_point_by_angle(lanemask)
        bounding_boxes = self.get_bounding_boxes(x_coords, y_coords)

        Transform2D.new_translation(Vector2.new())
        Lanemarking(Lanemarking.Style.SOLID, )
        pass

    def driveable_area_handler(self, ImageMsg):
        mask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        x_coords, y_coords = self.get_point_by_angle(mask)
        bounding_boxes = self.get_bounding_boxes(x_coords, y_coords)
        pass

    def get_bounding_boxes(self, x_coords, y_coords, epsilon=0.5, min_samples=100):
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

        # DBSCAN-Clustering
        clustering = DBSCAN(eps=epsilon, min_samples=min_samples).fit(points)
        labels = clustering.labels_

        # Berechnung der Bounding Boxes für jeden Cluster
        bounding_boxes = []
        unique_labels = set(labels)
        for label in unique_labels:
            if label == -1:  # Rauschen ignorieren
                continue
            cluster_points = points[labels == label]
            x_min, y_min = cluster_points.min(axis=0)
            x_max, y_max = cluster_points.max(axis=0)
            bounding_boxes.append((x_min, y_min, x_max, y_max))

        return bounding_boxes

    def get_point_by_angle(self, lanemask):  # nur einmal berrechen
        w = self.camera_width
        h = self.camera_height
        theta_h = np.deg2rad(self.camera_fov)  # Horizontaler FOV in Radiant
        theta_v = theta_h * (h / w)  # Vertikaler FOV in Radiant
        # Schritt 1: Pixelgitter erstellen
        u, v = np.meshgrid(np.arange(w), np.arange(h))

        # Schritt 2: Richtungsvektoren berechnen
        d_x = np.ones_like(u, dtype=float)  # x-Richtung bleibt konstant = 1
        d_y = (u - w / 2) / (w / 2) * np.tan(theta_h / 2)
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
        return x_ground, y_ground

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

    def create_Matrices(self):
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
        """
        Computes the rotation matrix from roll, pitch, and yaw angles.
        :param roll: Rotation around the x-axis (in degrees)
        :param pitch: Rotation around the y-axis (in degrees)
        :param yaw: Rotation around the z-axis (in degrees)
        :return: 3x3 rotation matrix
        """
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
        return R


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = lane_position("Lanedetection_node")
    node.run()

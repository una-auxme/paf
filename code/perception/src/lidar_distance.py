#!/usr/bin/env python
from joblib import Parallel, delayed
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge

# from mpl_toolkits.mplot3d import Axes3D
# from itertools import combinations
# from matplotlib.colors import LinearSegmentedColormap


class LidarDistance:
    """See doc/perception/lidar_distance_utility.md on
    how to configute this node
    """

    cluster_buffer = []

    def callback(self, data):
        """
        Callback-Funktion, die LiDAR-Punktwolkendaten verarbeitet.

        Führt Clustering und Bildberechnungen für die Punktwolken aus.

        :param data: LiDAR-Punktwolken als ROS PointCloud2-Nachricht.
        """

        self.start_clustering(data)
        self.start_image_calculation(data)

    def listener(self):
        """
        Initialisiert die ROS-Node, erstellt Publisher/Subscriber und hält sie aktiv.
        """
        rospy.init_node("lidar_distance")
        self.bridge = CvBridge()  # OpenCV-Bridge für Bildkonvertierungen

        # Publisher für gefilterte Punktwolken
        self.pub_pointcloud = rospy.Publisher(
            rospy.get_param(
                "~point_cloud_topic",
                "/carla/hero/" + rospy.get_namespace() + "_filtered",
            ),
            PointCloud2,
            queue_size=1,
        )

        # Publisher für Distanzbilder in verschiedene Richtungen
        self.dist_array_center_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Center/dist_array"),
            ImageMsg,
            queue_size=10,
        )
        self.dist_array_back_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Back/dist_array"),
            ImageMsg,
            queue_size=10,
        )
        self.dist_array_lidar_publisher = rospy.Publisher(
            rospy.get_param(
                "~image_distance_topic_cluster", "/paf/hero/dist_clustered"
            ),
            PointCloud2,
            queue_size=10,
        )
        rospy.loginfo("dist_array_lidar_publisher erfolgreich erstellt.")
        self.dist_array_left_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Left/dist_array"),
            ImageMsg,
            queue_size=10,
        )
        self.dist_array_right_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Right/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        # Subscriber für LiDAR-Daten (Punktwolken)
        rospy.Subscriber(
            rospy.get_param("~source_topic", "/carla/hero/LIDAR"),
            PointCloud2,
            self.callback,
        )

        rospy.loginfo("Lidar Processor Node gestartet.")
        rospy.spin()

    def start_clustering(self, data):
        """
        Filtert LiDAR-Punktwolken, führt Clustering durch und veröffentlicht die kombinierten Cluster.

        :param data: LiDAR-Punktwolken im ROS PointCloud2-Format.
        """

        # Punktwolken filtern, um irrelevante Daten zu entfernen
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        filtered_coordinates = coordinates[
            ~(
                (-2 <= coordinates["x"])
                & (coordinates["x"] <= 2)
                & (-1 <= coordinates["y"])
                & (coordinates["y"] <= 1)
            )  # Ausschluss von Punkten die das eigene Auto betreffen
            & (
                coordinates["z"] > -1.7 + 0.05
            )  # Mindesthöhe in z, um die Straße nicht zu clustern
        ]

        # Cluster-Daten aus den gefilterten Koordinaten berechnen
        clustered_points = cluster_lidar_data_from_pointcloud(
            coordinates=filtered_coordinates
        )

        # Nur gültige Cluster-Daten speichern
        if clustered_points:
            LidarDistance.cluster_buffer.append(clustered_points)
        else:
            rospy.logwarn("Keine Cluster-Daten erzeugt.")

        # Cluster kombinieren
        combined_clusters = combine_clusters(LidarDistance.cluster_buffer)

        LidarDistance.cluster_buffer = []

        # Veröffentliche die kombinierten Cluster
        self.publish_clusters(combined_clusters, data.header)

    def publish_clusters(self, combined_clusters, data_header):
        """
        Veröffentlicht kombinierte Cluster als ROS PointCloud2-Nachricht.

        :param combined_clusters: Kombinierte Punktwolken der Cluster als strukturiertes NumPy-Array.
        :param data_header: Header-Informationen der ROS-Nachricht.
        """
        # Konvertiere zu PointCloud2-Nachricht
        pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(combined_clusters)
        pointcloud_msg.header = data_header
        pointcloud_msg.header.stamp = rospy.Time.now()
        # Cluster veröffentlichen
        self.dist_array_lidar_publisher.publish(pointcloud_msg)

    def start_image_calculation(self, data):
        """
        Berechnet Distanzbilder basierend auf LiDAR-Daten und veröffentlicht sie.

        :param data: LiDAR-Punktwolken im ROS PointCloud2-Format.
        """
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        # Bildverarbeitung auf den Koordinaten durchführen
        processed_images = {
            "Center": None,
            "Back": None,
            "Left": None,
            "Right": None,
        }
        processed_images["Center"] = self.calculate_image_center(coordinates)
        processed_images["Back"] = self.calculate_image_back(coordinates)
        processed_images["Left"] = self.calculate_image_left(coordinates)
        processed_images["Right"] = self.calculate_image_right(coordinates)

        self.publish_images(processed_images, data.header)

    def calculate_image_center(self, coordinates):
        """
        Berechnet ein Distanzbild für die zentrale Ansicht aus LiDAR-Koordinaten.

        :param coordinates: Gefilterte LiDAR-Koordinaten als NumPy-Array.
        :return: Distanzbild als 2D-Array.
        """
        reconstruct_bit_mask_center = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=np.inf,
            min_x=0.0,
            min_z=-1.6,
        )
        reconstruct_coordinates_center = coordinates[reconstruct_bit_mask_center]
        reconstruct_coordinates_xyz_center = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_center, "intensity"
            ).tolist()
        )
        dist_array_center = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_center, focus="Center"
        )
        return dist_array_center

    def calculate_image_back(self, coordinates):
        # Back
        reconstruct_bit_mask_back = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=0.0,
            min_x=-np.inf,
            min_z=-1.6,
        )
        reconstruct_coordinates_back = coordinates[reconstruct_bit_mask_back]
        reconstruct_coordinates_xyz_back = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_back, "intensity"
            ).tolist()
        )
        dist_array_back = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_back, focus="Back"
        )
        return dist_array_back

    def calculate_image_left(self, coordinates):
        # Left
        reconstruct_bit_mask_left = lidar_filter_utility.bounding_box(
            coordinates,
            max_y=np.inf,
            min_y=0.0,
            min_z=-1.6,
        )
        reconstruct_coordinates_left = coordinates[reconstruct_bit_mask_left]
        reconstruct_coordinates_xyz_left = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_left, "intensity"
            ).tolist()
        )
        dist_array_left = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_left, focus="Left"
        )
        return dist_array_left

    def calculate_image_right(self, coordinates):
        # Right
        reconstruct_bit_mask_right = lidar_filter_utility.bounding_box(
            coordinates, max_y=-0.0, min_y=-np.inf, min_z=-1.6
        )
        reconstruct_coordinates_right = coordinates[reconstruct_bit_mask_right]
        reconstruct_coordinates_xyz_right = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_right, "intensity"
            ).tolist()
        )
        dist_array_right = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_right, focus="Right"
        )
        return dist_array_right

    def publish_images(self, processed_images, data_header):
        """
        Veröffentlicht Distanzbilder für verschiedene Richtungen als ROS-Bildnachrichten.

        :param processed_images: Dictionary mit Richtungen ("Center", "Back", etc.) als Schlüssel und Bildarrays als Werte.
        :param data_header: Header der ROS-Bildnachrichten.
        """
        # Nur gültige NumPy-Arrays weiterverarbeiten
        for direction, image_array in processed_images.items():
            if not isinstance(image_array, np.ndarray):
                continue

            # Konvertiere das Bild in eine ROS-Image-Nachricht
            dist_array_msg = self.bridge.cv2_to_imgmsg(
                image_array, encoding="passthrough"
            )
            dist_array_msg.header = data_header

            if direction == "Center":
                self.dist_array_center_publisher.publish(dist_array_msg)
            if direction == "Back":
                self.dist_array_back_publisher.publish(dist_array_msg)
            if direction == "Left":
                self.dist_array_left_publisher.publish(dist_array_msg)
            if direction == "Right":
                self.dist_array_right_publisher.publish(dist_array_msg)

    def reconstruct_img_from_lidar(self, coordinates_xyz, focus):
        """
        Rekonstruiert ein 2D-Bild aus 3D-LiDAR-Daten für eine gegebene Kameraansicht.

        :param coordinates_xyz: 3D-Koordinaten der gefilterten LiDAR-Punkte.
        :param focus: Kameraansicht (z. B. "Center", "Back").
        :return: Rekonstruiertes Bild als 2D-Array.
        """

        # Erstelle die intrinsische Kamera-Matrix basierend auf den Bildparametern (FOV, Auflösung)
        im = np.identity(3)
        im[0, 2] = 1280 / 2.0  # x-Verschiebung (Bildmitte)
        im[1, 2] = 720 / 2.0  # y-Verschiebung (Bildmitte)
        im[0, 0] = im[1, 1] = 1280 / (
            2.0 * np.tan(100 * np.pi / 360.0)
        )  # Skalierungsfaktor basierend auf FOV

        # Erstelle die extrinsische Kamera-Matrix (Identität für keine Transformation)
        ex = np.zeros(shape=(3, 4))
        ex[0][0] = ex[1][1] = ex[2][2] = 1
        m = np.matmul(im, ex)  # Kombiniere intrinsische und extrinsische Matrix

        # Initialisiere leere Bilder für die Rekonstruktion
        img = np.zeros(shape=(720, 1280), dtype=np.float32)
        dist_array = np.zeros(shape=(720, 1280, 3), dtype=np.float32)

        # Verarbeite jeden Punkt in der Punktwolke
        for c in coordinates_xyz:
            if focus == "Center":
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(
                    m, point
                )  # Projiziere 3D-Punkt auf 2D-Bildkoordinaten
                x, y = int(pixel[0] / pixel[2]), int(
                    pixel[1] / pixel[2]
                )  # Normalisiere Koordinaten
                if x >= 0 and x <= 1280 and y >= 0 and y <= 720:  # Prüfe Bildgrenzen
                    img[719 - y][1279 - x] = c[0]  # Setze Tiefenwert
                    dist_array[719 - y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            if focus == "Back":  # Berechne Bild für die Rückansicht
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y < 720:
                    img[y][1279 - x] = -c[0]
                    dist_array[y][1279 - x] = np.array(
                        [-c[0], c[1], c[2]], dtype=np.float32
                    )

            if focus == "Left":  # Berechne Bild für die linke Ansicht
                point = np.array([c[0], c[2], c[1], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y <= 720:
                    img[719 - y][1279 - x] = c[1]
                    dist_array[y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            if focus == "Right":  # Berechne Bild für die rechte Ansicht
                point = np.array([c[0], c[2], c[1], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x < 1280 and y >= 0 and y < 720:
                    img[y][1279 - x] = -c[1]
                    dist_array[y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

        return dist_array


def array_to_pointcloud2(points, header="hero/Lidar"):
    """
    Konvertiert ein Array von Punkten in eine ROS PointCloud2-Nachricht.

    :param points: Array von Punkten mit [x, y, z]-Koordinaten.
    :param header: Header-Informationen der ROS PointCloud2-Nachricht.
    :return: ROS PointCloud2-Nachricht.
    """
    # Sicherstellen, dass die Eingabe ein NumPy-Array ist
    points_array = np.array(points)

    # Konvertiere die Punkte in ein strukturiertes Array mit Feldern "x", "y", "z"
    points_structured = np.array(
        [(p[0], p[1], p[2]) for p in points_array],
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")],
    )

    # Erstelle eine PointCloud2-Nachricht aus dem strukturierten Array
    pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(points_structured)

    # Setze den Zeitstempel und den Header der Nachricht
    pointcloud_msg.header.stamp = rospy.Time.now()
    pointcloud_msg.header = header

    return pointcloud_msg


def get_largest_cluster(clustered_points, return_structured=True):
    """
    Ermittelt das größte Cluster aus gegebenen Punktwolken-Clustern.

    :param clustered_points: Dictionary mit Cluster-IDs und zugehörigen Punktwolken.
    :param return_structured: Gibt ein strukturiertes NumPy-Array zurück, falls True.
    :return: Größtes Cluster als NumPy-Array (roh oder strukturiert).
    """
    # Prüfen, ob es Cluster gibt
    if not clustered_points:
        return np.array([])

    # Identifiziere das größte Cluster basierend auf der Anzahl der Punkte
    largest_cluster_id, largest_cluster = max(
        clustered_points.items(), key=lambda item: len(item[1])
    )

    # Sicherstellen, dass das größte Cluster nicht leer ist
    if largest_cluster.size == 0:
        return np.array([])

    rospy.loginfo(
        f"Largest cluster: {largest_cluster_id} with {largest_cluster.shape[0]} points"
    )

    # Rohdaten zurückgeben, wenn kein strukturiertes Array benötigt wird
    if not return_structured:
        return largest_cluster

    # Konvertiere das größte Cluster in ein strukturiertes Array
    points_structured = np.empty(
        largest_cluster.shape[0], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]
    )
    points_structured["x"] = largest_cluster[:, 0]
    points_structured["y"] = largest_cluster[:, 1]
    points_structured["z"] = largest_cluster[:, 2]

    return points_structured


def combine_clusters(cluster_buffer):
    """
    Kombiniert Cluster aus mehreren Punktwolken zu einem strukturierten NumPy-Array.

    :param cluster_buffer: Liste von Dictionaries mit Cluster-IDs und Punktwolken.
    :return: Kombiniertes strukturiertes NumPy-Array mit Feldern "x", "y", "z", "cluster_id".
    """
    points_list = []
    cluster_ids_list = []

    for clustered_points in cluster_buffer:
        for cluster_id, points in clustered_points.items():
            if points.size > 0:  # Ignoriere leere Cluster
                points_list.append(points)
                # Erstelle ein Array mit der Cluster-ID für alle Punkte des Clusters
                cluster_ids_list.append(
                    np.full(points.shape[0], cluster_id, dtype=np.float32)
                )

    if not points_list:  # Falls keine Punkte vorhanden sind
        return np.array(
            [], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cluster_id", "f4")]
        )

    # Kombiniere alle Punkte und Cluster-IDs in zwei separate Arrays
    all_points = np.vstack(points_list)
    all_cluster_ids = np.concatenate(cluster_ids_list)

    # Erstelle ein strukturiertes Array für die kombinierten Daten
    combined_points = np.zeros(
        all_points.shape[0],
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cluster_id", "f4")],
    )
    combined_points["x"] = all_points[:, 0]
    combined_points["y"] = all_points[:, 1]
    combined_points["z"] = all_points[:, 2]
    combined_points["cluster_id"] = all_cluster_ids

    return combined_points


def cluster_lidar_data_from_pointcloud(coordinates, eps=0.3, min_samples=10):
    """
    Führt Clustering auf LiDAR-Daten mit DBSCAN durch und gibt Cluster zurück.

    :param coordinates: LiDAR-Punktwolken als NumPy-Array mit "x", "y", "z".
    :param eps: Maximaler Abstand zwischen Punkten, um sie zu einem Cluster zuzuordnen.
    :param min_samples: Minimale Anzahl von Punkten, um ein Cluster zu bilden.
    :return: Dictionary mit Cluster-IDs und zugehörigen Punktwolken.
    """
    if coordinates.shape[0] == 0:
        rospy.logerr("Das Eingabe-Array 'coordinates' ist leer.")
        return {}

    # Extrahiere x, y und z aus den Koordinaten für die DBSCAN-Berechnung
    xyz = np.column_stack((coordinates["x"], coordinates["y"], coordinates["z"]))

    if xyz.shape[0] == 0:
        rospy.logwarn("Keine Datenpunkte für DBSCAN verfügbar. Überspringe Clustering.")
        return {}

    # Wende DBSCAN an, um Cluster-Labels für die Punktwolke zu berechnen
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(xyz)
    labels = clustering.labels_

    # Entferne Rauschen (Cluster-ID: -1) und bestimme gültige Cluster-IDs
    unique_labels = np.unique(labels)
    valid_labels = unique_labels[unique_labels != -1]

    # Erstelle ein Dictionary mit Cluster-IDs und den zugehörigen Punkten
    clusters = Parallel(n_jobs=-1)(
        delayed(lambda l: (l, xyz[labels == l]))(label) for label in valid_labels
    )
    clusters = dict(clusters)

    return clusters


if __name__ == "__main__":
    """
    Initialisiert die LidarDistance-Klasse und startet die Listener-Methode.
    """
    lidar_distance = LidarDistance()
    lidar_distance.listener()

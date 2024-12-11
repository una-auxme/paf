#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from sklearn.cluster import DBSCAN
from cv_bridge import CvBridge
from joblib import Parallel, delayed
import queue
import threading

# from mpl_toolkits.mplot3d import Axes3D
# from itertools import combinations
# from matplotlib.colors import LinearSegmentedColormap


class LidarDistance:
    """See doc/perception/lidar_distance_utility.md on
    how to configute this node
    """

    # Klassenattribute
    cluster_queue = queue.Queue()
    image_queue = queue.Queue()
    workers_initialized = False
    cluster_data_list = []

    def callback(self, data):
        """Callback function, filters a PontCloud2 message
            by restrictions defined in the launchfile.

            Publishes a Depth image for the specified camera angle.
            Each angle has do be delt with differently since the signs of the
            coordinate system change with the view angle.

        :param data: a PointCloud2
        """

        if not LidarDistance.workers_initialized:
            LidarDistance.workers_initialized = True
            self.start_image_worker()
            self.start_cluster_worker()

        if not self.cluster_thread.is_alive:
            rospy.logwarn("Cluster-Worker abgestürzt. Neustart...")
            self.start_cluster_worker()

        if not self.image_thread.is_alive:
            rospy.logwarn("Image-Worker abgestürzt. Neustart...")
            self.start_image_worker()

        LidarDistance.cluster_queue.put(data)
        if LidarDistance.cluster_queue.qsize() > 10:
            rospy.logwarn(
                "Cluster-Worker kommt nicht hinterher. Queue size:"
                + str(LidarDistance.cluster_queue.qsize()),
            )
        # Dasselbe Koordinatenset für die Bildberechnung in die Bild-Warteschlange legen
        LidarDistance.image_queue.put(data)
        if LidarDistance.image_queue.qsize() > 10:
            rospy.logwarn(
                f"Image-Worker ({self.image_thread.is_alive}) kommt nicht hinterher. Queue size: {str(LidarDistance.image_queue.qsize())}"
            )

    def listener(self):
        """
        Initializes the node and it's publishers
        """
        # run simultaneously.
        rospy.init_node("lidar_distance")
        self.bridge = CvBridge()

        self.pub_pointcloud = rospy.Publisher(
            rospy.get_param(
                "~point_cloud_topic",
                "/carla/hero/" + rospy.get_namespace() + "_filtered",
            ),
            PointCloud2,
            queue_size=10,
        )

        # publisher for dist_array
        self.dist_array_center_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Center/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        # publisher for dist_array
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

        # publisher for dist_array
        self.dist_array_left_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Left/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        # publisher for dist_array
        self.dist_array_right_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Right/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        rospy.Subscriber(
            rospy.get_param("~source_topic", "/carla/hero/LIDAR"),
            PointCloud2,
            self.callback,
        )

        rospy.loginfo("Lidar Processor Node gestartet.")
        rospy.spin()

    def cluster_worker(self):
        """
        Worker für das Clustern von Punkten.
        """
        while True:
            try:
                # Neueste Aufgabe holen
                data = LidarDistance.cluster_queue.get(timeout=1)

                # Restliche Aufgaben aus der Queue entfernen
                discarded_tasks = 0
                while not LidarDistance.cluster_queue.empty():
                    LidarDistance.cluster_queue.get()
                    discarded_tasks += 1

                # Anzahl verworfener Einträge loggen
                if discarded_tasks > 0:
                    rospy.logwarn(f"{discarded_tasks} Einträge wurden verworfen.")

                # Stop-Signal überprüfen
                if data is None:  # Stop-Signal
                    break

                # Punktwolken-Daten verarbeiten
                coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
                filtered_coordinates = coordinates[(coordinates["x"] >= 2)]
                clustered_points = cluster_lidar_data_from_pointcloud(
                    coordinates=filtered_coordinates
                )

                # Kombinierte Cluster erstellen
                cluster_structured = combine_clusters(clustered_points)

                # Konvertiere zu PointCloud2-Nachricht
                pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(
                    cluster_structured
                )
                pointcloud_msg.header = data.header
                pointcloud_msg.header.stamp = rospy.Time.now()

                # Cluster veröffentlichen
                self.dist_array_lidar_publisher.publish(pointcloud_msg)

            except queue.Empty:
                rospy.loginfo("Cluster-Queue ist leer. Warte auf neue Aufgaben.")
                continue  # Zurück zur Queue-Warte

            except Exception as e:
                rospy.logerr(f"Fehler im Cluster-Worker: {e}")

    def image_worker(self):
        """
        Worker für die Berechnung von Bildern.
        """
        while True:
            data = LidarDistance.image_queue.get()
            if data is None:  # Stop-Signal
                break

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

    def start_cluster_worker(self):
        self.cluster_thread = threading.Thread(target=self.cluster_worker, daemon=True)
        self.cluster_thread.start()

    def start_image_worker(self):
        self.image_thread = threading.Thread(target=self.image_worker, daemon=True)
        self.image_thread.start()

    def calculate_image_center(self, coordinates):
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
        for direction, image_array in processed_images.items():
            if not isinstance(image_array, np.ndarray):
                continue
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
        reconstruct 3d LIDAR-Data and calculate 2D Pixel
        according to Camera-World

        Args:
            coordinates_xyz (np.array): filtered lidar points
            focus (String): Camera Angle

        Returns:
            image: depth image for camera angle
        """

        # intrinsic matrix for camera:
        # width -> 300, height -> 200, fov -> 100 (agent.py)
        im = np.identity(3)
        im[0, 2] = 1280 / 2.0
        im[1, 2] = 720 / 2.0
        im[0, 0] = im[1, 1] = 1280 / (2.0 * np.tan(100 * np.pi / 360.0))

        # extrinsic matrix for camera
        ex = np.zeros(shape=(3, 4))
        ex[0][0] = 1
        ex[1][1] = 1
        ex[2][2] = 1
        m = np.matmul(im, ex)

        # reconstruct camera image with LIDAR-Data
        img = np.zeros(shape=(720, 1280), dtype=np.float32)
        dist_array = np.zeros(shape=(720, 1280, 3), dtype=np.float32)
        for c in coordinates_xyz:
            # center depth image
            if focus == "Center":
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y <= 720:
                    img[719 - y][1279 - x] = c[0]
                    dist_array[719 - y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            # back depth image
            if focus == "Back":
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y < 720:
                    img[y][1279 - x] = -c[0]
                    dist_array[y][1279 - x] = np.array(
                        [-c[0], c[1], c[2]], dtype=np.float32
                    )

            # left depth image
            if focus == "Left":
                point = np.array([c[0], c[2], c[1], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y <= 720:
                    img[719 - y][1279 - x] = c[1]
                    dist_array[y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            # right depth image
            if focus == "Right":
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
    points_array = np.array(points)

    points_structured = np.array(
        [(p[0], p[1], p[2]) for p in points_array],
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")],
    )

    pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(points_structured)

    pointcloud_msg.header.stamp = rospy.Time.now()
    pointcloud_msg.header = header

    return pointcloud_msg


def get_largest_cluster(clustered_points, return_structured=True):
    """
    Bestimmt das größte Cluster aus den gegebenen Punkten und gibt es zurück.

    :param clustered_points: Dictionary mit Cluster-IDs als Schlüssel und Punktwolken als Werte.
    :param return_structured: Gibt ein strukturiertes Array zurück, falls True.
    :return: Größtes Cluster als Array (entweder raw oder strukturiert).
    """
    if not clustered_points:
        rospy.logerr(
            "Clustered points are empty. Cannot determine the largest cluster."
        )
        return np.array([])

    # Finde das größte Cluster direkt
    largest_cluster_id, largest_cluster = max(
        clustered_points.items(), key=lambda item: len(item[1])
    )

    if largest_cluster.size == 0:
        rospy.logerr(f"Largest cluster (ID {largest_cluster_id}) is empty.")
        return np.array([])

    rospy.loginfo(
        f"Largest cluster: {largest_cluster_id} with {largest_cluster.shape[0]} points"
    )

    # Falls kein strukturiertes Array benötigt wird, direkt zurückgeben
    if not return_structured:
        return largest_cluster

    # Andernfalls Punkte in ein strukturiertes Array konvertieren
    points_structured = np.empty(
        largest_cluster.shape[0], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]
    )
    points_structured["x"] = largest_cluster[:, 0]
    points_structured["y"] = largest_cluster[:, 1]
    points_structured["z"] = largest_cluster[:, 2]

    return points_structured


def combine_clusters(clustered_points):
    """
    Kombiniert Cluster-Daten in ein strukturiertes Array.

    :param clustered_points: Dictionary mit Cluster-IDs als Schlüssel und 3D-Punktwolken als Werten.
    :return: Kombiniertes strukturiertes NumPy-Array mit Feldern "x", "y", "z", "cluster_id".
    """

    # Vorab die Gesamtanzahl der Punkte berechnen
    total_points = sum(points.shape[0] for points in clustered_points.values())

    # Erstelle ein leeres strukturiertes Array
    combined_points = np.empty(
        total_points,
        dtype=[("x", "f4"), ("y", "f4"), ("z", "f4"), ("cluster_id", "f4")],
    )

    # Fülle das Array direkt
    index = 0
    for cluster_id, points in clustered_points.items():
        num_points = points.shape[0]
        combined_points["x"][index : index + num_points] = points[:, 0]
        combined_points["y"][index : index + num_points] = points[:, 1]
        combined_points["z"][index : index + num_points] = points[:, 2]
        combined_points["cluster_id"][index : index + num_points] = cluster_id
        index += num_points

    return combined_points


def fuse_clusters(cluster_data_list):
    """
    Fusioniert eine Liste von Cluster-Daten und entfernt redundante Punkte basierend auf (x, y, z).

    :param cluster_data_list: Liste von NumPy-Arrays mit Feldern (x, y, z, intensity)
    :return: Fusioniertes NumPy-Array mit einzigartigen Punkten
    """
    # Kombiniere alle Arrays in ein großes Array
    concatenated_points = np.concatenate(cluster_data_list, axis=0)

    # Erstelle eine flache Ansicht für die Felder (x, y, z)
    unique_xyz, indices = np.unique(
        concatenated_points[["x", "y", "z"]].view(np.float32).reshape(-1, 3),
        axis=0,
        return_index=True,
    )

    # Behalte die entsprechenden `intensity`-Werte basierend auf den eindeutigen Indizes
    unique_points = concatenated_points[indices]

    return unique_points


def cluster_lidar_data_from_pointcloud(coordinates, eps=0.1, min_samples=10):
    """
    Cluster LiDAR-Daten effizient mit DBSCAN.
    """
    # Erstelle Matrix direkt mit column_stack (schneller als vstack)
    xyz = np.column_stack((coordinates["x"], coordinates["y"], coordinates["z"]))

    # DBSCAN-Cluster-Berechnung
    clustering = DBSCAN(eps=eps, min_samples=min_samples).fit(xyz)
    labels = clustering.labels_

    # Nur eindeutige Cluster (Rauschen entfernen)
    unique_labels = np.unique(labels)
    valid_labels = unique_labels[unique_labels != -1]

    # Parallelisiertes Extrahieren der Cluster
    clusters = Parallel(n_jobs=-1)(
        delayed(lambda l: (l, xyz[labels == l]))(label) for label in valid_labels
    )
    clusters = dict(clusters)

    return clusters


if __name__ == "__main__":
    lidar_distance = LidarDistance()
    lidar_distance.listener()

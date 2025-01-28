import rospy
from sensor_msgs.msg import Imu, PointCloud2
from visualization_msgs.msg import Marker
import numpy as np
import tf
import math
from ros_compatibility.node import CompatibleNode
import time
from coordinate_transformation import quat_to_heading


class ImuFilter(CompatibleNode):
    def __init__(self):
        rospy.init_node("ground_filter_node", anonymous=True)

        # Parameter für Schwellenwerte und Debugging
        self.pitch_threshold = rospy.get_param(
            "~pitch_threshold", 0.1
        )  # Toleranz für Pitch
        self.roll_threshold = rospy.get_param(
            "~roll_threshold", 0.1
        )  # Toleranz für Roll
        self.debug = True
        self.role_name = self.get_param("role_name", "hero")

        # Subscriptions
        rospy.Subscriber(
            "/carla/" + self.role_name + "/IMU",
            Imu,
            self.imu_callback,
        )
        # rospy.Subscriber(
        #    "/carla/hero/RADAR0",
        #    PointCloud2,
        #    self.radar_callback,
        #    callback_args="RADAR0",
        # )
        # rospy.Subscriber(
        #    "/carla/hero/RADAR1",
        #    PointCloud2,
        #    self.radar_callback,
        #    callback_args="RADAR1",
        # )
        # self.lidar_sub = rospy.Subscriber(
        #    "/lidar/points", PointCloud2, self.lidar_callback
        # )

        # Publishers
        self.filtered_radar_pub = rospy.Publisher(
            "/paf/hero/IMU/radar/filtered_points", PointCloud2, queue_size=10
        )
        self.filtered_lidar_pub = rospy.Publisher(
            "/paf/hero/IMU/lidar/filtered_points", PointCloud2, queue_size=10
        )
        self.marker_pub = rospy.Publisher(
            "/paf/hero/IMU/ground_filter/debug_marker", Marker, queue_size=10
        )

        # Variablen für aktuelle IMU-Daten
        self.current_pitch = 0.0
        self.current_roll = 0.0

        # Buffer to check noise
        self.pitch_buffer = []
        self.roll_buffer = []

        # Initialisierung der Variablen
        self.current_pitch = 0.0  # Aktueller Pitch-Winkel (in Rad)
        self.last_time = time.time()  # Zeitstempel der letzten Berechnung

        # Schwellenwert für statische Situationen
        self.acceleration_threshold = 0.2  # m/s²

        self.previous_time = time.time()
        self.pitch = 0.0  # Initialer Pitch-Winkel

    def imu_callback(self, msg):
        """Verarbeitet die IMU-Daten und extrahiert Pitch und Roll."""
        # orientation_q = msg.orientation
        # orientation_euler = tf.transformations.euler_from_quaternion(
        #    [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        # )
        # self.current_pitch = orientation_euler[1]  # Pitch
        # self.current_roll = orientation_euler[0]  # Roll

        """# Extrahiere IMU-Daten
        accel_x = msg.linear_acceleration.x
        accel_z = msg.linear_acceleration.z
        gyro_y = msg.angular_velocity.y

        # Zeitdifferenz berechnen
        current_time = time.time()
        delta_time = current_time - self.last_time
        self.last_time = current_time

        # Berechnung der Bewegungsbeschleunigung
        total_acceleration = math.sqrt(accel_x**2 + accel_z**2)

        # Wenn die Bewegungsbeschleunigung gering ist, auf die Gravitation verlassen
        if total_acceleration < self.acceleration_threshold:
            # Berechne den Pitch aus der Gravitation (Beschleunigungsmethode)
            pitch_acc = math.atan2(accel_x, accel_z)
            self.current_pitch = pitch_acc
        else:
            # Berechne den Pitch durch Integration der Gyroskopdaten
            self.current_pitch += gyro_y * delta_time

        # Save pitch and roll for debugging
        self.pitch_buffer.append(self.current_pitch)
        self.roll_buffer.append(self.current_roll)"""

        accel_x = msg.linear_acceleration.x
        accel_z = msg.linear_acceleration.z

        # Verhindere Division durch Null
        if accel_z == 0:
            accel_z = 1e-6

        # Berechne Pitch-Winkel (in Radiant)
        self.current_pitch = math.atan2(accel_x, accel_z)
        self.current_pitch *= 0.25

        # Debug-Marker für RViz veröffentlichen
        if self.debug:
            self.publish_ground_projection_marker()

    def radar_callback(self, msg):
        """Filtert Radar-Daten basierend auf Neigungswinkeln."""
        filtered_points = self.filter_points(msg)
        self.filtered_radar_pub.publish(filtered_points)

    def lidar_callback(self, msg):
        """Filtert Lidar-Daten basierend auf Neigungswinkeln."""
        filtered_points = self.filter_points(msg)
        self.filtered_lidar_pub.publish(filtered_points)

    def filter_points(self, pointcloud_msg):
        """
        Filtert Punkte aus einem PointCloud2 basierend auf Pitch,
        Fahrzeughöhe und Distanz.

        Parameter:
        - pointcloud_msg: sensor_msgs/PointCloud2
            Die eingehenden Radar-Punkte.

        Rückgabe:
        - sensor_msgs/PointCloud2
            Gefilterte Radar-Punkte, bei denen keine Bodenreflexion vorliegt.
        """
        # Konvertiere PointCloud2 in numpy-Array
        points = pointcloud2_to_array(
            pointcloud_msg
        )  # Erwartet ein Nx3/Nx4-Array mit [x, y, z, ...]

        filtered_points = []
        radar_height = 1  # Höhe des Radars über dem Boden (in Metern)

        # Berechnung der Schwellenwerte basierend auf Pitch
        pitch_rad = self.current_pitch  # Pitch-Winkel in Radiant
        pitch_slope = math.tan(pitch_rad)  # Steigung durch Pitch

        # Filterlogik: Prüfe, ob der Punkt unterhalb der berechneten Bodenhöhe liegt
        if points is None:
            return

        mask = points[:, 2] < points[:, 0] * pitch_slope + radar_height

        filtered_points = points[mask]

        """for point in points:
            x, y, z = point[:3]  # Extrahiere die Koordinaten
            ground_height_at_x = radar_height - (
                x * pitch_slope
            )  # Z-Höhe des Bodens bei Distanz X

            if z > ground_height_at_x + self.z_tolerance:  # Toleranz für Filterung
                filtered_points.append(
                    point
                )  # Punkt beibehalten, wenn er nicht am Boden liegt"""

        # Konvertiere gefilterte Punkte zurück in PointCloud2
        return array_to_pointcloud2(np.array(filtered_points), pointcloud_msg.header)

    def publish_ground_projection_marker(self):
        """
        Veröffentlicht Marker zur Visualisierung der berechneten Bodenhöhe
        und zeigt den aktuellen Pitch-Winkel als Text in RViz an.
        """
        # Pfeilmarker für den Neigungswinkel
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "hero"
        arrow_marker.header.stamp = rospy.Time.now()
        arrow_marker.ns = "ground_filter"
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        arrow_marker.scale.x = 2.0  # Länge des Pfeils
        arrow_marker.scale.y = 0.1  # Breite des Pfeils
        arrow_marker.scale.z = 0.1  # Höhe des Pfeils

        arrow_marker.color.r = 1.0
        arrow_marker.color.g = 0.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        # Orientierung des Pfeils basierend auf Pitch
        quaternion = tf.transformations.quaternion_from_euler(
            self.current_roll, self.current_pitch, 0
        )
        arrow_marker.pose.orientation.x = quaternion[0]
        arrow_marker.pose.orientation.y = quaternion[1]
        arrow_marker.pose.orientation.z = quaternion[2]
        arrow_marker.pose.orientation.w = quaternion[3]

        # Textmarker für den Pitch-Winkel
        text_marker = Marker()
        text_marker.header.frame_id = "hero"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "ground_filter"
        text_marker.id = 2
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD

        text_marker.scale.z = 0.5  # Textgröße
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
        text_marker.color.a = 1.0

        # Textinhalt: Pitch-Winkel in Grad
        pitch_in_degrees = math.degrees(self.current_pitch)
        text_marker.text = f"Pitch: {pitch_in_degrees:.2f}°"

        # Position des Textmarkers (über dem Pfeil)
        text_marker.pose.position.x = 0.0
        text_marker.pose.position.y = 0.0
        text_marker.pose.position.z = 2.0  # Über dem Fahrzeug anzeigen

        # Veröffentlichen der Marker
        self.marker_pub.publish(arrow_marker)
        self.marker_pub.publish(text_marker)


def pointcloud2_to_array(msg):
    """Hilfsfunktion: Konvertiert PointCloud2 in ein numpy-Array."""
    # Implementierung hängt von der genauen Struktur ab
    pass


def array_to_pointcloud2(array, header):
    """Hilfsfunktion: Konvertiert numpy-Array in PointCloud2."""
    # Implementierung hängt von der genauen Struktur ab
    pass


if __name__ == "__main__":
    try:
        node = ImuFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

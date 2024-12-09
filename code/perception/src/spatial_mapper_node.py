#!/usr/bin/env python3

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
        try:
            # Daten und Layout extrahieren
            flat_data = np.array(msg.data, dtype=np.int8)
            dimensions = [dim.size for dim in msg.layout.dim]

            # Daten zurück in die ursprüngliche Form bringen
            if len(dimensions) == 3:
                segmentation_array = flat_data.reshape(dimensions)
            else:
                rospy.logwarn("Unexpected dimensions in segmentation mask")

            self.process_segmentation_mask(segmentation_array)

        except Exception as e:
            rospy.logerr(f"Error processing segmentation mask: {e}")

    def process_segmentation_mask(self, segmentation_array):
        """
        Verarbeitung der Segmentierungsmaske:
        - Identifiziert Klassen in der Maske
        - Erstellt Marker für jede Klasse
        """
        # MarkerArray zum Sammeln aller Marker
        marker_array = MarkerArray()

        marker_id = 0
        for segmentation_mask in segmentation_array:
            # Calculate depth values from self.dist_array
            depth_array = self.calculate_depth_values(segmentation_mask)

            non_zero_values = depth_array[depth_array > 0]
            if len(non_zero_values) == 0:
                continue
            min_value = np.min(non_zero_values)
            closest_point_index = np.where(depth_array == min_value)
            closest_point = self.dist_array[closest_point_index][0]

            # Linken, Rechten und Nähsten Punkt bekommen
            # left_depth, right_depth, near_depth = self.extract_percentile_points(
            #    segmentation_mask, self.dist_array
            # )

            # Marker für diese Klasse erstellen
            # point = np.random.uniform(-5, 5, size=3)  # Zufällige Position im Raum
            obj_class = 2
            marker = self.get_marker(
                point=closest_point, id=marker_id, obj_class=obj_class
            )
            marker_array.markers.append(marker)
            marker_id += 1

        # MarkerArray veröffentlichen
        try:
            self.marker_publisher.publish(marker_array)
            rospy.loginfo(f"Published {len(marker_array.markers)} markers.")
        except Exception as e:
            rospy.logerr(f"Error publishing markers: {e}")

    def get_marker(self, point, id, obj_class):
        c_color = carla_colors[obj_class]
        marker = Marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "min_values"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 1.0
        marker.color.r = c_color[0]
        marker.color.g = c_color[1]
        marker.color.b = c_color[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = -point[2]
        marker.lifetime = rospy.Duration(0.5)
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

    def extract_percentile_points(mask, depth, percent=20):
        # Extrahiere Spalten-Indices basierend auf der Maske
        cols = np.any(mask > 0, axis=0)
        col_indices = np.where(cols)[0]
        if len(col_indices) == 0:
            return [], [], []

        # Teile in linkeste, rechteste und finde die nähesten Punkte
        num_cols = len(col_indices)
        left_20_indices = col_indices[: max(1, num_cols * percent // 100)]
        right_20_indices = col_indices[-max(1, num_cols * percent // 100) :]

        # Maskiere relevante Punkte
        left_20_mask = np.zeros_like(mask, dtype=bool)
        left_20_mask[:, left_20_indices] = mask[:, left_20_indices] > 0

        right_20_mask = np.zeros_like(mask, dtype=bool)
        right_20_mask[:, right_20_indices] = mask[:, right_20_indices] > 0

        # Näheste 20 % basierend auf Depth-Werten
        depth_values = depth[mask > 0]
        if len(depth_values) == 0:
            return [], [], []
        depth_threshold = np.percentile(depth_values, percent)
        near_20_mask = (depth <= depth_threshold) & (mask > 0)

        return (depth[left_20_mask], depth[right_20_mask], depth[near_20_mask])


if __name__ == "__main__":
    roscomp.init("SpatialMapperNode")
    node = SpatialMapperNode("SpatialMapperNode")
    node.run()

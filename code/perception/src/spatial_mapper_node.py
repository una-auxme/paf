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
        self.dist_arrays = dist_array
        rospy.loginfo("Received new depth image")

    def segmentation_mask_callback(self, msg):
        """
        Callback-Funktion zum Empfangen und Verarbeiten der Segmentierungsdaten
        """
        try:
            # Daten und Layout extrahieren
            flat_data = np.array(msg.data, dtype=np.int8)
            dimensions = [dim.size for dim in msg.layout.dim]

            # Daten zurück in die ursprüngliche Form bringen
            if len(dimensions) == 3:  # Beispiel für (Klassen, Höhe, Breite)
                segmentation_array = flat_data.reshape(dimensions)
            else:
                rospy.logwarn("Unexpected dimensions in segmentation mask")

            rospy.loginfo(
                f"Received segmentation mask with shape: {segmentation_array.shape}"
            )

            # Hier können Sie weitere Verarbeitung durchführen
            self.process_segmentation_mask(segmentation_array)

        except Exception as e:
            rospy.logerr(f"Error processing segmentation mask: {e}")

    def process_segmentation_mask(self, segmentation_array):
        """
        Verarbeitung der Segmentierungsmaske:
        - Identifiziert Klassen in der Maske
        - Erstellt Marker für jede Klasse
        """
        rospy.loginfo("Processing segmentation mask...")

        # MarkerArray zum Sammeln aller Marker
        marker_array = MarkerArray()

        # Klassen identifizieren (non-zero values in der Maske)
        unique_classes = np.unique(segmentation_array)
        rospy.loginfo(f"Unique classes in segmentation mask: {unique_classes}")

        marker_id = 0
        for obj_class in unique_classes:
            if obj_class == 0:  # Klasse 0 ist der Hintergrund, überspringen
                continue

            # Dummy-Punkt für den Marker (Testzwecke)
            point = np.random.uniform(-5, 5, size=3)  # Zufällige Position im Raum

            # Marker für diese Klasse erstellen
            marker = self.get_marker(point=point, id=marker_id, obj_class=obj_class)
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


if __name__ == "__main__":
    roscomp.init("SpatialMapperNode")
    node = SpatialMapperNode("SpatialMapperNode")
    node.run()

#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from std_msgs.msg import Int8MultiArray, Float32MultiArray
from cv_bridge import CvBridge
import numpy as np
import rospy
import ros_numpy


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
        self.setup_pointcloud_publisher()
        self.setup_segmentation_mask_subscriber()
        self.setup_object_distance_publishers()

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

    def setup_object_distance_publishers(self):
        """
        sets up a publisher to publish a list of objects
        and their distances
        """

        self.distance_publisher = self.new_publisher(
            msg_type=Float32MultiArray,
            topic=f"/paf/{self.role_name}/{self.side}/object_distance",
            qos_profile=1,
        )

    def setup_pointcloud_publisher(self):
        """
        sets up a publisher for visualization pointcloud
        """

        self.pointcloud_publisher = self.new_publisher(
            msg_type=PointCloud2,
            topic=f"/paf/{self.role_name}/visualization_pointcloud",
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
        nonzero_mask_classes = segmentation_array != 0
        first_non_zero_indices = np.argmax(
            nonzero_mask_classes.reshape(segmentation_array.shape[0], -1), axis=1
        )
        row_indices, col_indices = np.unravel_index(
            first_non_zero_indices, segmentation_array.shape[1:]
        )
        unique_classes = segmentation_array[
            np.arange(segmentation_array.shape[0]), row_indices, col_indices
        ]

        # Stack depth values for all segmentation masks
        tiled_dist_array = np.tile(
            self.dist_array, (segmentation_array.shape[0], 1, 1, 1)
        )
        tiled_dist_array[..., 2] += 1.7

        stacked_depth_values = self.calculate_depth_values(
            segmentation_array, tiled_dist_array
        )
        valid_mask = stacked_depth_values > 0
        non_zero_mask = (
            ~(tiled_dist_array[..., 0] == 0.0)
            & ~(tiled_dist_array[..., 1] == 0.0)
            & ~(tiled_dist_array[..., 2] == 1.7)
        )
        z_filter_mask = tiled_dist_array[..., 2] >= 0.3
        car_length = 4.9
        car_width = 1.86436
        filter_hero_car_mask = (
            (tiled_dist_array[..., 0] >= -car_length / 2)
            & (tiled_dist_array[..., 0] <= car_length / 2)
            & (tiled_dist_array[..., 1] >= -car_width / 2)
            & (tiled_dist_array[..., 1] <= car_width / 2)
        )

        combined_mask = (
            valid_mask & non_zero_mask & z_filter_mask & ~filter_hero_car_mask
        )

        # Combine all valid points from the segmentation masks
        valid_points = tiled_dist_array[combined_mask]

        if valid_points.size > 0:
            combined_points = np.zeros(
                valid_points.shape[0], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]
            )
            combined_points["x"] = valid_points[:, 0]
            combined_points["y"] = valid_points[:, 1]
            combined_points["z"] = valid_points[:, 2]

            # Calculate a distance_output array for the distance_publisher
            # It holds the class, the min_x and the min_abs_y distance
            # get minimum x and y point distance for each segmentation_mask
            mask_indices = np.argwhere(combined_mask)
            distance_output = []
            classes_array = unique_classes[mask_indices[:, 0]]
            x_coords = valid_points[:, 0]
            y_coords = valid_points[:, 1]

            distance_output = np.column_stack(
                (classes_array, x_coords, y_coords)
            ).flatten()

            if len(distance_output) > 0:
                self.distance_publisher.publish(
                    Float32MultiArray(data=np.array(distance_output))
                )

        else:
            combined_points = np.array(
                [], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]
            )

        pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(combined_points)
        pointcloud_msg.header.frame_id = "hero"
        pointcloud_msg.header.stamp = rospy.Time.now()
        self.pointcloud_publisher.publish(pointcloud_msg)

    def run(self):
        self.spin()

    def calculate_depth_values(self, segmentation_array, tiled_dist_array):
        """
        Berechnet die Tiefenwerte basierend auf der Maske und den Lidar-Daten
        """
        obj_dist = np.abs(tiled_dist_array)  # Kopie der Distanzwerte
        output = np.zeros_like(
            segmentation_array, dtype=np.float32
        )  # Leeres Array in Maskenform
        mask = segmentation_array.astype(bool)  # Konvertiere Maske in boolesches Format
        abs_distance = np.sqrt(
            obj_dist[..., 0] ** 2 + obj_dist[..., 1] ** 2 + obj_dist[..., 2] ** 2
        )
        output[mask] = abs_distance[mask]  # Werte nur an Maskenpositionen setzen
        return output


if __name__ == "__main__":
    roscomp.init("SpatialMapperNode")
    node = SpatialMapperNode("SpatialMapperNode")
    node.run()

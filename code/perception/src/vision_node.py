#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from sklearn.cluster import DBSCAN
import torch
import cv2
from vision_node_helper import coco_to_carla, carla_colors
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import PointCloud2, Image as ImageMsg
from std_msgs.msg import (
    Header,
    Float32MultiArray,
    Int8MultiArray,
)
from cv_bridge import CvBridge
from torchvision.utils import draw_segmentation_masks
import numpy as np
from ultralytics import NAS, YOLO, RTDETR, SAM, FastSAM
import rospy
from ultralytics.utils.ops import scale_masks
from time import time_ns
import ros_numpy
from visualization_msgs.msg import MarkerArray
from lidar_distance import create_bounding_box_marker


class VisionNode(CompatibleNode):
    """
    VisionNode:

    The Vision-Node provides advanced object-detection features.
    It can handle different camera angles, easily switch between
    pretrained models and distances of objects.

    Advanced features are limited to ultralytics models and center view.
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # dictionary of pretrained models
        self.model_dict = {
            "yolov8n": (YOLO, "yolov8n.pt", "detection", "ultralytics"),
            "yolov8s": (YOLO, "yolov8s.pt", "detection", "ultralytics"),
            "yolov8m": (YOLO, "yolov8m.pt", "detection", "ultralytics"),
            "yolov8l": (YOLO, "yolov8l.pt", "detection", "ultralytics"),
            "yolov8x": (YOLO, "yolov8x.pt", "detection", "ultralytics"),
            "yolo_nas_l": (NAS, "yolo_nas_l.pt", "detection", "ultralytics"),
            "yolo_nas_m": (NAS, "yolo_nas_m.pt", "detection", "ultralytics"),
            "yolo_nas_s": (NAS, "yolo_nas_s.pt", "detection", "ultralytics"),
            "rtdetr-l": (RTDETR, "rtdetr-l.pt", "detection", "ultralytics"),
            "rtdetr-x": (RTDETR, "rtdetr-x.pt", "detection", "ultralytics"),
            "yolov8x-seg": (YOLO, "yolov8x-seg.pt", "segmentation", "ultralytics"),
            "sam_l": (SAM, "sam_l.pt", "detection", "ultralytics"),
            "FastSAM-x": (FastSAM, "FastSAM-x.pt", "detection", "ultralytics"),
            "yolo11n-seg": (YOLO, "yolo11n-seg.pt", "segmentation", "ultralytics"),
            "yolo11s-seg": (YOLO, "yolo11s-seg.pt", "segmentation", "ultralytics"),
            "yolo11m-seg": (YOLO, "yolo11m-seg.pt", "segmentation", "ultralytics"),
            "yolo11l-seg": (YOLO, "yolo11l-seg.pt", "segmentation", "ultralytics"),
        }

        # general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.center = self.get_param("center")
        self.back = self.get_param("back")
        self.left = self.get_param("left")
        self.right = self.get_param("right")

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.depth_images = []
        self.dist_array = None
        self.lidar_array = None

        # publish / subscribe setup
        if self.center:
            self.setup_camera_subscriptions("Center")
        if self.back:
            self.setup_camera_subscriptions("Back")
        if self.left:
            self.setup_camera_subscriptions("Left")
        if self.right:
            self.setup_camera_subscriptions("Right")

        # self.setup_rainbow_subscription()
        self.setup_dist_array_subscription()
        self.setup_pointcloud_publisher()
        self.setup_camera_publishers()
        self.setup_object_distance_publishers()
        self.setup_traffic_light_publishers()
        self.setup_image_marker_publishers()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        # model setup
        model_info = self.model_dict[self.get_param("model")]
        self.model = model_info[0]
        self.weights = model_info[1]
        self.type = model_info[2]
        self.framework = model_info[3]
        self.save = True

        # print configuration of Vision-Node
        print("Vision Node Configuration:")
        print("Device -> ", self.device)
        print(f"Model -> {self.get_param('model')},")
        print(f"Type -> {self.type}, Framework -> {self.framework}")

        # ultralytics setup
        if self.framework == "ultralytics":
            self.model = self.model(self.weights)
        else:
            rospy.logerr("Framework not supported")

    def setup_camera_subscriptions(self, side):
        """
        sets up a subscriber to the selected camera angle

        Args:
            side (String): Camera angle specified in launch file
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic=f"/carla/{self.role_name}/{side}/image",
            qos_profile=1,
        )

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

    def setup_pointcloud_publisher(self):
        """
        sets up a publisher for visualization pointcloud
        """

        self.pointcloud_publisher = self.new_publisher(
            msg_type=PointCloud2,
            topic=f"/paf/{self.role_name}/visualization_pointcloud",
            qos_profile=1,
        )

    def setup_camera_publishers(self):
        """
        sets up a publisher to the selected camera angle
        multiple publishers are used since the vision node could handle
        multiple camera angles at the same time if enough
        resources are available
        """

        if self.center:
            self.publisher_center = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Center/segmented_image",
                qos_profile=1,
            )
        if self.back:
            self.publisher_back = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Back/segmented_image",
                qos_profile=1,
            )
        if self.left:
            self.publisher_left = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Left/segmented_image",
                qos_profile=1,
            )
        if self.right:
            self.publisher_right = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Right/segmented_image",
                qos_profile=1,
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

    def setup_traffic_light_publishers(self):
        """
        sets up a publisher for traffic light detection
        """

        self.traffic_light_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/{self.side}/segmented_traffic_light",
            qos_profile=1,
        )

    def setup_image_marker_publishers(self):
        self.marker_visualization_vision_node_publisher = rospy.Publisher(
            rospy.get_param("~marker_topic", "/paf/hero/Image/Marker"),
            MarkerArray,
            queue_size=10,
        )

    def handle_camera_image(self, image):
        """
        This function handles a new camera image and publishes the
        calculated visualization according to the correct camera angle

        Args:
            image (image msg): Image from camera scubscription
        """

        # free up cuda memory
        if self.device == "cuda":
            torch.cuda.empty_cache()

        if self.framework == "ultralytics":
            timestamp1 = time_ns()

            vision_result = self.predict_ultralytics(
                image, return_image=False, image_size=320
            )
            rospy.loginfo(
                f"Segmentation mask processing took {(time_ns() - timestamp1) / 1e6} ms"
            )
        else:
            # As we will not use pytorch models this is to prevent errors
            rospy.logerr("Framework not supported")
            return
        if vision_result is None:
            return
        # publish vision result to rviz
        img_msg = self.bridge.cv2_to_imgmsg(vision_result, encoding="bgr8")
        img_msg.header = image.header

        # publish img to corresponding angle topic
        header_id = rospy.resolve_name(img_msg.header.frame_id)
        if (
            "Center" in header_id
            or "Back" in header_id
            or "Left" in header_id
            or "Right" in header_id
        ):
            side = header_id.split("/")[2]
            if side == "Center":
                self.publisher_center.publish(img_msg)
            if side == "Back":
                self.publisher_back.publish(img_msg)
            if side == "Left":
                self.publisher_left.publish(img_msg)
            if side == "Right":
                self.publisher_right.publish(img_msg)

    def handle_dist_array(self, dist_array):
        """
        This function overwrites the current lidar depth image from
        the lidar distance node with the latest depth image.
        The function also calculates the depth values of the lidar

        Args:
            dist_array (image msg): Depth image frim Lidar Distance Node
        """
        # callback function for lidar depth image
        # since frequency is lower than image frequency
        # the latest lidar image is saved
        lidar_array = self.bridge.imgmsg_to_cv2(
            img_msg=dist_array, desired_encoding="passthrough"
        )
        lidar_array_copy = np.copy(lidar_array)
        # add camera height to the z-axis
        lidar_array_copy[..., 2] += 1.7
        self.lidar_array = lidar_array_copy
        self.dist_array = self.calculate_depth_values(lidar_array_copy)

    def predict_ultralytics(self, image, return_image=True, image_size=640):
        """
        This function takes in an image from a camera, predicts
        an ultralytics model on the image and looks for lidar points
        in the bounding boxes.

        This function also implements a visualization
        of what has been calculated for RViz.

        Args:
            image (image msg): image from camera subsription

        Returns:
            (cv image): visualization output for rvizw
        """
        scaled_masks = None
        # preprocess image
        cv_image = self.bridge.imgmsg_to_cv2(
            img_msg=image, desired_encoding="passthrough"
        )
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # run model prediction

        output = self.model.track(
            cv_image, half=True, verbose=False, imgsz=image_size  # type: ignore
        )
        if not (
            hasattr(output[0], "masks")
            and output[0].masks is not None
            and hasattr(output[0], "boxes")
            and output[0].boxes is not None
            and self.dist_array is not None
            and self.lidar_array is not None
        ):
            return None

        carla_classes = np.array(coco_to_carla)[
            output[0].boxes.cls.to(torch.int).numpy()  # type: ignore
        ]

        masks = torch.tensor(output[0].masks.data)
        scaled_masks = scale_masks(
            masks.unsqueeze(1), cv_image.shape[:2], True
        ).squeeze(1)
        valid_points, class_indices = self.process_segmentation_mask(
            scaled_masks.cpu().numpy(),
            carla_classes=carla_classes,
            distance_array=self.dist_array,
            lidar_array=self.lidar_array,
        )
        if valid_points is not None:
            try:
                clustered_points, valid_labels, valid_class_indices = (
                    self.cluster_points(valid_points, class_indices)
                )
                self.publish_pointcloud(clustered_points)

                bounding_boxes = self.calculate_bounding_boxes(
                    clustered_points, valid_class_indices
                )

                # Create a MarkerArray for visualization
                marker_array = MarkerArray()
                for label in bounding_boxes:
                    marker = create_bounding_box_marker(
                        label, bounding_boxes[label], frame_id="hero"
                    )
                    marker_array.markers.append(marker)

                # Publish the MarkerArray for visualization
                self.marker_visualization_vision_node_publisher.publish(marker_array)
            except Exception as e:
                rospy.logerr(f"Error while processing point clusters: {e}")

        # proceed with traffic light detection
        # if 9 in output[0].boxes.cls:
        #    self.process_traffic_lights(output[0], cv_image, image.header)

        if return_image is False or scaled_masks is None:
            return None

        transposed_image = np.transpose(cv_image, (2, 0, 1))
        image_tensor = torch.tensor(transposed_image, dtype=torch.uint8)
        masks_tensor = torch.tensor(scaled_masks > 0, dtype=torch.bool)

        class_colors = np.array(carla_colors)[carla_classes].tolist()
        drawn_images = draw_segmentation_masks(
            image_tensor, masks_tensor, alpha=0.6, colors=class_colors
        )

        np_image = drawn_images.permute(1, 2, 0).cpu().numpy()
        return cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)

    def process_segmentation_mask(
        self, segmentation_array, carla_classes, distance_array, lidar_array
    ):
        # Only process the segmentation mask if the distance array is not None
        valid_distances_mask = distance_array > 0
        car_length = 4.9
        car_width = 1.86436
        # Filter out points that are not in the car and not on the road and are not zero
        car_filter_mask = (
            # filter out points that are not in the car
            (lidar_array[..., 0] >= -car_length / 2)
            & (lidar_array[..., 0] <= car_length / 2)
            & (lidar_array[..., 1] >= -car_width / 2)
            & (lidar_array[..., 1] <= car_width / 2)
        )
        # Filter out points that are on the road
        road_filter_mask = lidar_array[..., 2] >= 0.3
        # Filter out points that are zero
        zero_filter_mask = (
            ~(lidar_array[..., 0] == 0.0)
            & ~(lidar_array[..., 1] == 0.0)
            & ~(lidar_array[..., 2] == 1.7)
        )
        lidar_filter_mask = ~car_filter_mask & road_filter_mask & zero_filter_mask
        combined_mask = valid_distances_mask & lidar_filter_mask
        # tiled_mask holds all the valid points
        valid_points_from_mask = (
            segmentation_array.astype(bool) & combined_mask[None, ...]
        )
        # get the x, y, z values of the valid points
        valid_indices = np.nonzero(valid_points_from_mask)
        valid_points = lidar_array[valid_indices[1], valid_indices[2]]
        combined_points = None
        if valid_points.size > 0:
            combined_points = np.zeros(
                valid_points.shape[0], dtype=[("x", "f4"), ("y", "f4"), ("z", "f4")]
            )
            combined_points["x"], combined_points["y"], combined_points["z"] = (
                valid_points.T
            )
            self.publish_distance_output(
                valid_points, valid_points_from_mask, carla_classes
            )

        return combined_points, valid_indices[0]

    def cluster_points(self, points, class_indices, eps=0.5, min_samples=2):
        """
        Clusters all points in the point cloud and determines the largest cluster for each segmentation class in one pass.

        Parameters:
            points (numpy structured array): Array of points with fields 'x', 'y', 'z'.
            class_indices (numpy array): Array of segmentation mask indices for each point.
            eps (float): Maximum distance between points to be considered in the same neighborhood.
            min_samples (int): Minimum number of points to form a dense region (cluster).

        Returns:
            clustered_points (numpy structured array): Points belonging to the largest cluster for each class index.
            valid_labels (numpy array): Labels corresponding to each class index (one per class).
            valid_class_indices (numpy array): Class indices corresponding to the returned points.
        """
        if points.size == 0:
            return np.array([], dtype=points.dtype), [], []

        # Convert to a regular 2D array for clustering
        xyz_points = np.vstack((points["x"], points["y"], points["z"])).T

        # Apply DBSCAN clustering to all points at once
        db = DBSCAN(eps=eps, min_samples=min_samples)
        cluster_labels = db.fit_predict(xyz_points)

        # Combine class indices and cluster labels to identify unique groups
        combined_labels = np.vstack((class_indices, cluster_labels)).T

        # Ignore noise points (cluster_label == -1)
        valid_mask = cluster_labels != -1
        valid_points = points[valid_mask]
        valid_combined_labels = combined_labels[valid_mask]
        valid_class_indices = class_indices[valid_mask]

        # Find the largest cluster for each class
        unique_combinations, inverse_indices = np.unique(
            valid_combined_labels, axis=0, return_inverse=True
        )
        counts = np.bincount(inverse_indices)

        # Map the largest cluster for each class
        largest_clusters = {}
        for idx, (class_idx, cluster_label) in enumerate(unique_combinations):
            if (
                class_idx not in largest_clusters
                or counts[idx] > counts[largest_clusters[class_idx]]
            ):
                largest_clusters[class_idx] = idx

        # Extract points belonging to the largest cluster for each class
        selected_points_mask = np.isin(
            inverse_indices,
            [largest_clusters[class_idx] for class_idx in largest_clusters],
        )
        clustered_points = valid_points[selected_points_mask]
        valid_labels = list(largest_clusters.keys())

        return clustered_points, valid_labels, valid_class_indices[selected_points_mask]

    def calculate_bounding_boxes(self, clustered_points, cluster_labels):
        """
        Calculates axis-aligned bounding boxes (AABBs) for clustered points.

        Parameters:
            clustered_points (numpy structured array): Array of clustered points with
            fields 'x', 'y', 'z'.
            cluster_labels (numpy array): Array of cluster labels corresponding to each
            point.

        Returns:
            bounding_boxes (dict): A dictionary where each key is a cluster label, and
            the value is a dictionary:
                {
                    'min': (min_x, min_y, min_z),
                    'max': (max_x, max_y, max_z)
                }
        """
        bounding_boxes = {}

        # Get unique cluster labels (excluding noise, label = -1)
        unique_labels = set(cluster_labels)
        if -1 in unique_labels:
            unique_labels.remove(-1)  # Exclude noise

        for label in unique_labels:
            # Extract points belonging to the current cluster
            cluster_points = clustered_points[cluster_labels == label]

            # Calculate min and max for each axis
            min_x, min_y, min_z = (
                cluster_points["x"].min(),
                cluster_points["y"].min(),
                cluster_points["z"].min(),
            )
            max_x, max_y, max_z = (
                cluster_points["x"].max(),
                cluster_points["y"].max(),
                cluster_points["z"].max(),
            )

            # Store bounding box as a dictionary
            bounding_boxes[label] = min_x, max_x, min_y, max_y, min_z, max_z

        return bounding_boxes

    def publish_pointcloud(self, combined_points):
        # Publish point cloud
        pointcloud_msg = ros_numpy.point_cloud2.array_to_pointcloud2(combined_points)
        pointcloud_msg.header.frame_id = "hero"
        pointcloud_msg.header.stamp = rospy.Time.now()
        self.pointcloud_publisher.publish(pointcloud_msg)

    def publish_distance_output(
        self, valid_points, valid_points_from_mask, carla_classes
    ):
        """
        Publishes the distance output of the object detection
        """
        # Direct calculation of distance output
        mask_indices = np.argwhere(valid_points_from_mask)
        classes_array = np.array(carla_classes)[mask_indices[:, 0]]
        distance_output = np.column_stack(
            (classes_array, valid_points[:, 0], valid_points[:, 1])
        ).ravel()

        if distance_output.size > 0:
            self.distance_publisher.publish(Float32MultiArray(data=distance_output))

    def calculate_depth_values(self, dist_array):
        """
        Berechnet die Tiefenwerte basierend auf den Lidar-Daten
        """
        abs_distance = np.sqrt(
            dist_array[..., 0] ** 2 + dist_array[..., 1] ** 2 + dist_array[..., 2] ** 2
        )
        return abs_distance

    async def process_traffic_lights(self, prediction, cv_image, image_header):
        indices = (prediction.boxes.cls == 9).nonzero().squeeze().cpu().numpy()
        indices = np.asarray([indices]) if indices.size == 1 else indices

        max_y = 360  # middle of image
        min_prob = 0.30

        for index in indices:
            box = prediction.boxes.cpu().data.numpy()[index]

            if box[4] < min_prob:
                continue

            if (box[2] - box[0]) * 1.5 > box[3] - box[1]:
                continue  # ignore horizontal boxes

            if box[1] > max_y:
                continue

            box = box[0:4].astype(int)
            segmented = cv_image[box[1] : box[3], box[0] : box[2]]

            traffic_light_y_distance = box[1]

            traffic_light_image = self.bridge.cv2_to_imgmsg(segmented, encoding="rgb8")
            traffic_light_image.header = image_header
            traffic_light_image.header.frame_id = str(traffic_light_y_distance)
            self.traffic_light_publisher.publish(traffic_light_image)

    def run(self):
        self.spin()
        pass


if __name__ == "__main__":
    roscomp.init("VisionNode")
    node = VisionNode("VisionNode")
    node.run()

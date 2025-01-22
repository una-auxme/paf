#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from sklearn.cluster import DBSCAN
import torch
import cv2
from vision_node_helper import coco_to_carla, carla_colors
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Float32MultiArray
from cv_bridge import CvBridge
from torchvision.utils import draw_segmentation_masks
import numpy as np
from ultralytics import YOLO
import rospy
from ultralytics.utils.ops import scale_masks
from mapping.msg import ClusteredPointsArray
from perception_utils import array_to_clustered_points

from time import time_ns
from copy import deepcopy
import asyncio


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
            "yolo11n-seg": (YOLO, "yolo11n-seg.pt", "segmentation", "ultralytics"),
            "yolo11x-seg": (YOLO, "yolo11l-seg.pt", "segmentation", "ultralytics"),
        }

        self.timesum = 0
        self.timenum = 0
        self.timestart = False

        # general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.view_camera = self.get_param("view_camera")
        self.camera_resolution = self.get_param("camera_resolution")

        self.depth_images = []
        self.dist_array = None
        self.lidar_array = None

        self.setup_subscriber()
        self.setup_publisher()
        self.setup_model()

    def setup_subscriber(self):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic=f"/carla/{self.role_name}/Zoom/image",
            qos_profile=1,
        )

    def setup_publisher(self):
        """
        sets up all publishers for the Vision-Node
        """

        self.traffic_light_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Zoom/segmented_traffic_light",
            qos_profile=1,
        )

    def setup_model(self):
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        model_info = self.model_dict[self.get_param("model")]
        self.model = model_info[0]
        self.weights = model_info[1]
        self.type = model_info[2]
        self.framework = model_info[3]
        self.save = True

        print("Zoom Node Configuration:")
        print("Device -> ", self.device)
        print(f"Model -> {self.get_param('model')},")
        print(f"Type -> {self.type}, Framework -> {self.framework}")

        if self.framework == "ultralytics":
            self.model = self.model(self.weights)
        else:
            rospy.logerr("Framework not supported")

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

        # time_start = time_ns()
        vision_result = self.predict_ultralytics(
            image, return_image=self.view_camera, image_size=self.camera_resolution
        )
        # time_delta = (time_ns() - time_start) / 1e6
        # if self.timestart:
        #     self.timesum += time_delta
        #     self.timenum += 1
        #     printTime = self.timesum / self.timenum
        #     rospy.loginfo(
        #         f"runtime: {printTime} ms   timesum: {self.timesum} timenum: {self.timenum}"
        #     )
        # elif time_delta < 90:
        #     self.timestart = True

        if vision_result is None:
            return

        # publish vision result to rviz
        img_msg = self.bridge.cv2_to_imgmsg(vision_result, encoding="bgr8")
        img_msg.header = image.header
        self.publisher_center.publish(img_msg)

    def predict_ultralytics(self, image, return_image=True, image_size=720):
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
            output[0].boxes.cls.to(torch.int).cpu().numpy()  # type: ignore
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
        if valid_points is None or valid_points.size == 0:
            return None
        clustered_points, cluster_indices, carla_classes_indices = self.cluster_points(
            valid_points, class_indices, carla_classes
        )
        if clustered_points is None or clustered_points.size == 0:
            return None
        try:
            self.publish_distance_output(clustered_points, carla_classes_indices)
            clustered_lidar_points_msg = array_to_clustered_points(
                clustered_points,
                cluster_indices,
                object_class_array=carla_classes_indices,
            )
            self.pointcloud_publisher.publish(clustered_lidar_points_msg)
        except Exception as e:
            rospy.logerr(f"Error in publishing pointcloud: {e}")
            return None
        # proceed with traffic light detection
        if 9 in output[0].boxes.cls:
            asyncio.run(
                self.process_traffic_lights(output[0], cv_image, deepcopy(image.header))
            )

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
        return valid_points, valid_indices[0]

    async def process_traffic_lights(self, prediction, cv_image, image_header):
        indices = (prediction.boxes.cls == 9).nonzero().squeeze().cpu().numpy()
        indices = np.asarray([indices]) if indices.size == 1 else indices

        max_y = 360  # middle of image
        min_prob = 0.030

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

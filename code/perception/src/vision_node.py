#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import torch
from torchvision.models.segmentation import (
    DeepLabV3_ResNet101_Weights,
    deeplabv3_resnet101,
)
from torchvision.models.detection.faster_rcnn import (
    FasterRCNN_MobileNet_V3_Large_320_FPN_Weights,
    FasterRCNN_ResNet50_FPN_V2_Weights,
    fasterrcnn_resnet50_fpn_v2,
    fasterrcnn_mobilenet_v3_large_320_fpn,
)
import torchvision.transforms as t
import cv2
from vision_node_helper import coco_to_carla, get_carla_color
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import (
    Header,
    Float32MultiArray,
    Int8MultiArray,
    MultiArrayLayout,
    MultiArrayDimension,
)
from cv_bridge import CvBridge
from torchvision.utils import draw_bounding_boxes, draw_segmentation_masks
import numpy as np
from ultralytics import NAS, YOLO, RTDETR, SAM, FastSAM
import rospy
from ultralytics.utils.ops import scale_masks
from time import time_ns


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
            "frcnn_resnet50_fpn_v2": (
                fasterrcnn_resnet50_fpn_v2(
                    weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT
                ),
                FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT,
                "detection",
                "pyTorch",
            ),
            "frcnn_mobilenet_v3_large_320_fpn": (
                fasterrcnn_mobilenet_v3_large_320_fpn(
                    weights=FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT
                ),
                FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT,
                "detection",
                "pyTorch",
            ),
            "deeplabv3_resnet101": (
                deeplabv3_resnet101(weights=DeepLabV3_ResNet101_Weights.DEFAULT),
                DeepLabV3_ResNet101_Weights.DEFAULT,
                "segmentation",
                "pyTorch",
            ),
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
        self.dist_arrays = None

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
        self.setup_camera_publishers()
        self.setup_object_distance_publishers()
        self.setup_traffic_light_publishers()
        self.setup_segmentation_mask_publishers()
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
        torch.cuda.memory.set_per_process_memory_fraction(0.1)

        # pyTorch and CUDA setup
        if self.framework == "pyTorch":
            for param in self.model.parameters():
                param.requires_grad = False
                self.model.to(self.device)

        # ultralytics setup
        if self.framework == "ultralytics":
            self.model = self.model(self.weights)

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

    def setup_segmentation_mask_publishers(self):
        """
        sets up a publisher for segmentation masks found in the image
        """

        self.segmentation_mask_publisher = self.new_publisher(
            msg_type=numpy_msg(Int8MultiArray),
            topic=f"/paf/{self.role_name}/Center/segmentation_masks",
            qos_profile=1,
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

        vision_result = None

        if self.framework == "pyTorch":
            vision_result = self.predict_torch(image)

        if self.framework == "ultralytics":
            vision_result = self.predict_ultralytics(image, return_image=False)
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

    def predict_torch(self, image):
        """
        This function takes in an image from a camera and predicts a
        PyTorch model on the image. Depending on the the type of prediction
        a visualization function creates either a segmentation mask or bounding
        boxes for the image. The resulting image is returned.

        Args:
            image (image msg): image from a camera subscription

        Returns:
            image: visualization of prediction for rviz
        """

        # set model in evaluation mode
        self.model.eval()

        # preprocess image
        cv_image = self.bridge.imgmsg_to_cv2(
            img_msg=image, desired_encoding="passthrough"
        )
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        preprocess = t.Compose(
            [
                t.ToTensor(),
                t.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ]
        )
        input_image = preprocess(cv_image).unsqueeze(dim=0)

        # get prediction
        input_image = input_image.to(self.device)
        prediction = self.model(input_image)

        # apply visualition
        if self.type == "detection":
            vision_result = self.apply_bounding_boxes(cv_image, prediction[0])
        if self.type == "segmentation":
            vision_result = self.create_mask(cv_image, prediction["out"])

        return vision_result

    def predict_ultralytics(self, image, return_image=True):
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

        # preprocess image
        cv_image = self.bridge.imgmsg_to_cv2(
            img_msg=image, desired_encoding="passthrough"
        )
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        # run model prediction

        output = self.model(cv_image, half=True, verbose=False, imgsz=320)

        # handle distance of objects

        # set up lists for visualization of distance outputs
        c_colors = []
        boxes = output[0].boxes
        carla_classes = [coco_to_carla[int(cls)] for cls in boxes.cls]
        c_colors = [get_carla_color(int(cls)) for cls in boxes.cls]

        if hasattr(output[0], "masks") and output[0].masks is not None:
            masks = output[0].masks.data
            scaled_masks = scale_masks(
                masks.unsqueeze(1), cv_image.shape[:2], True
            ).squeeze(1)
            self.publish_segmentation_mask(scaled_masks, carla_classes)

        else:
            masks = None

        transposed_image = np.transpose(cv_image, (2, 0, 1))
        image_np_with_detections = torch.tensor(transposed_image, dtype=torch.uint8)

        # proceed with traffic light detection
        if 9 in output[0].boxes.cls:
            self.process_traffic_lights(output[0], cv_image, image.header)

        if return_image is False:
            return None

        if masks is not None:

            drawn_images = draw_segmentation_masks(
                image_np_with_detections,
                torch.from_numpy(scaled_masks > 0),
                alpha=0.6,
                colors=c_colors,
            )

        np_image = np.transpose(drawn_images.detach().numpy(), (1, 2, 0))
        return cv2.cvtColor(np_image, cv2.COLOR_BGR2RGB)

    def publish_segmentation_mask(self, scaled_masks, carla_classes):
        segmentation_array = (
            (scaled_masks > 0).cpu() * torch.tensor(carla_classes).view(-1, 1, 1)
        ).numpy()

        # Flaches Array und Layout erstellen

        layout = MultiArrayLayout()
        layout.dim = [
            MultiArrayDimension(
                label="classes", size=segmentation_array.shape[0], stride=1
            ),
            MultiArrayDimension(
                label="height",
                size=segmentation_array.shape[1],
                stride=segmentation_array.shape[1] * segmentation_array.shape[2],
            ),
            MultiArrayDimension(
                label="width",
                size=segmentation_array.shape[2],
                stride=segmentation_array.shape[2],
            ),
        ]
        layout.data_offset = 0

        # ROS-Nachricht erstellen
        msg = Int8MultiArray()
        msg.layout = layout
        timestamp1 = time_ns()
        msg.data = segmentation_array.ravel().tolist()
        timeStamp2 = time_ns()
        rospy.loginfo(f"Time for prediction: {(timeStamp2 - timestamp1) / 1e6} ms")
        # Nachricht veröffentlichen
        try:
            self.segmentation_mask_publisher.publish(msg)
        except Exception as e:
            rospy.logerr(f"Error publishing segmentation mask: {e}")

    def min_x(self, dist_array):
        """
        Calculate min x (distance forward)

        Args:
            dist_array (np array): numpy array containing all
            lidar point in one bounding box

        Returns:
            np.array: 1x3 numpy array of min x lidar point
        """

        min_x_sorted_indices = np.argsort(dist_array[:, :, 0], axis=None)
        x, y = np.unravel_index(min_x_sorted_indices[0], dist_array.shape[:2])
        return dist_array[x][y].copy()

    def min_abs_y(self, dist_array):
        """
        Calculate min abs y (distance sideways)

        Args:
            dist_array (np array): numpy array containing all
            lidar point in one bounding box

        Returns:
            np.array: 1x3 numpy array of min abs y lidar point
        """

        abs_distance_copy = np.abs(dist_array.copy())
        min_y_sorted_indices = np.argsort(abs_distance_copy[:, :, 1], axis=None)
        x, y = np.unravel_index(min_y_sorted_indices[0], abs_distance_copy.shape[:2])
        return dist_array[x][y].copy()

    # you can add similar functions to support other camera angles here

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

    def create_mask(self, input_image, model_output):
        """
        function to create segmentation mask for pyTorch models

        Args:
            input_image (np.array): original image
            model_output (np.array): model output

        Returns:
            np.array: image with segmentation mask
        """

        output_predictions = torch.argmax(model_output, dim=0)
        for i in range(21):
            output_predictions[i] = output_predictions[i] == i

        output_predictions = output_predictions.to(dtype=torch.bool)

        transposed_image = np.transpose(input_image, (2, 0, 1))
        tensor_image = torch.tensor(transposed_image)
        tensor_image = tensor_image.to(dtype=torch.uint8)
        segmented_image = draw_segmentation_masks(
            tensor_image, output_predictions, alpha=0.6
        )
        cv_segmented = segmented_image.detach().cpu().numpy()
        cv_segmented = np.transpose(cv_segmented, (1, 2, 0))
        return cv_segmented

    def apply_bounding_boxes(self, input_image, model_output):
        """
        function to draw bounding boxes for pyTorch models

        Args:
            input_image (np.array): original image
            model_output (np.array): model output

        Returns:
            np.array: image with segmentation mask
        """

        transposed_image = np.transpose(input_image, (2, 0, 1))
        image_np_with_detections = torch.tensor(transposed_image, dtype=torch.uint8)
        boxes = model_output["boxes"]
        labels = [self.weights.meta["categories"][i] for i in model_output["labels"]]

        box = draw_bounding_boxes(
            image_np_with_detections,
            boxes,
            labels,
            colors="blue",
            width=3,
            font_size=24,
        )

        np_box_img = np.transpose(box.detach().numpy(), (1, 2, 0))
        box_img = cv2.cvtColor(np_box_img, cv2.COLOR_BGR2RGB)
        return box_img

    def run(self):
        self.spin()
        pass


if __name__ == "__main__":
    roscomp.init("VisionNode")
    node = VisionNode("VisionNode")
    node.run()

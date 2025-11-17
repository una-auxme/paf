from typing import Optional
import cv2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np

import rclpy
from rclpy.node import Node

# for the lane detection model
import torch
from PIL import Image
import torchvision.transforms as t
from functools import partial
from typing import Any, cast


class Lanedetection_node(Node):
    """YOLOP:
    Model for Lanedetection and Driveable Area Detection

    subscribes to camera image and publishes lane masks and driveable area for further
    planning
    """

    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )

        # load model
        self.model = torch.hub.load("hustvl/yolop", "yolop", pretrained=True)

        self.device = torch.device("cuda")
        self.model.to(self.device)
        self.bridge = CvBridge()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"
        # Initialize dist_arrays to None
        self.dist_arrays = None

        # Latest lane and driveable area masks per camera
        self.lane_mask_left = None
        self.lane_mask_right = None
        self.driveable_area_left = None
        self.driveable_area_right = None
        self.left_yaw_correction_deg = 5.0
        self.right_yaw_correction_deg = -5.0
        self.left_x_shift_px = 40.0
        self.right_x_shift_px = -40.0

        # setup subscriptions
        self.setup_camera_subscriptions("Camera_right")
        self.setup_camera_subscriptions("Camera_left")
        self.setup_dist_array_subscription()
        # setup publishers
        self.setup_lane_publisher()
        self.setup_driveable_area_publisher()

        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def setup_camera_subscriptions(self, side):
        """
        sets up a subscriber to the selected camera angle

        Args:
            side (String): Camera angle specified in launch file
        """

        self.create_subscription(
            msg_type=ImageMsg,
            callback=partial(self.image_handler, side=side),
            topic=f"/carla/{self.role_name}/{side}/image",
            qos_profile=1,
        )

    def setup_dist_array_subscription(self):
        """
        sets up a subscription to the lidar distance array
        depth image of the selected camera angle
        """

        self.create_subscription(
            msg_type=ImageMsg,
            callback=self.handle_dist_array,
            topic=f"/paf/{self.role_name}/Center/dist_array",
            qos_profile=1,
        )

    def setup_lane_publisher(self):
        """sets up a publisher for the lane mask
        topic: /Center/lane_mask
        """
        self.lane_mask_publisher = self.create_publisher(
            msg_type=ImageMsg,
            topic=f"/paf/{self.role_name}/Center/lane_mask",
            qos_profile=1,
        )

    def setup_driveable_area_publisher(self):
        """sets up a publisher for the driveable area mask
        topic: /Center/lane_mask
        """
        self.driveable_area_publisher = self.create_publisher(
            msg_type=ImageMsg,
            topic=f"/paf/{self.role_name}/Center/driveable_area",
            qos_profile=1,
        )

    def image_handler(self, ImageMsg, side: str):
        """Callback function for image subscribers.

        Applies lane detection and driveable area detection to the given
        ImageMsg from one of the forward-facing cameras.

        Args:
            ImageMsg: ROS Image message from the camera.
            side (str): Which camera this image came from
                (e.g. "Camera_left" or "Camera_right").
        """
        # free up cuda memory
        if self.device.type == "cuda":
            torch.cuda.empty_cache()

        image, original_image = self.preprocess_image(ImageMsg)
        self.original_h, self.original_w, _ = original_image.shape
        with torch.no_grad():
            image = image.to(self.device)
            _, da_seg_out, ll_seg_out = self.detect_lanes(image)

        ll_seg_scaled, da_seg_scaled = self.postprocess_image(da_seg_out, ll_seg_out)

        ll_aligned = self.warp_mask_to_center(ll_seg_scaled, side)
        da_aligned = self.warp_mask_to_center(da_seg_scaled, side)

        # Store the latest masks for the corresponding camera
        if side == "Camera_left":
            self.lane_mask_left = ll_aligned
            self.driveable_area_left = da_aligned
        elif side == "Camera_right":
            self.lane_mask_right = ll_aligned
            self.driveable_area_right = da_aligned
        else:
            # Fallback for unexpected camera names: publish the single mask directly
            ros_driveable_area = self.bridge.cv2_to_imgmsg(da_seg_scaled)
            ros_lane_mask = self.bridge.cv2_to_imgmsg(ll_seg_scaled)
            self.lane_mask_publisher.publish(ros_lane_mask)
            self.driveable_area_publisher.publish(ros_driveable_area)
            return

        # Publish a fused lane/drivable-area mask using all available cameras
        self.publish_fused_masks()

    def warp_mask_to_center(self, mask: np.ndarray, side: str) -> Optional[np.ndarray]:
        if mask is None:
            return None

        h, w = mask.shape[:2]

        # ROTATION + SHIFT

        if side == "Camera_left":
            angle = -8.0  # turn to the right
            tx = 80  # move to the right
        else:  # Camera_right
            angle = 8.0  # turn to the left
            tx = -80  # move to the left

        M = cv2.getRotationMatrix2D((w / 2, h / 2), angle, 1.0)
        M[0, 2] += tx

        mask = cv2.warpAffine(
            mask,
            M,
            (w, h),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
            # type: ignore
        )

        # PERSPECTIVE CORRECTION
        # starting points
        pts1 = np.array(
            [[0.0, 0.0], [float(w), 0.0], [0.0, float(h)], [float(w), float(h)]],
            dtype=np.float32,
        )

        if side == "Camera_left":
            # Drag the upper left corner to the RIGHT
            pts2_list: list[list[float]] = [
                [float(w * 0.025), 0.0],
                [float(w * 0.95), 0.0],
                [float(w * 0.7), float(h)],
                [float(w), float(h)],
            ]
        else:  # Camera_right
            # Drag the upper right corner to the LEFT
            pts2_list: list[list[float]] = [
                [float(w * 0.05), 0.0],
                [float(w * 0.75), 0.0],
                [float(w * 0.3), float(h)],
                [float(w), float(h)],
            ]

            pts2 = cast(np.ndarray, np.array(pts2_list, dtype=np.float32))

        H = cv2.getPerspectiveTransform(pts1, pts2)  # type: ignore

        mask = cv2.warpPerspective(
            mask,
            H,
            (w, h),
            flags=cv2.INTER_NEAREST,
            borderMode=cv2.BORDER_CONSTANT,
            borderValue=0,
            # type: ignore
        )

        return mask

    def publish_fused_masks(self):
        """Fuse lane and driveable-area masks from left and right cameras.

        This implements option C (overlap fusion without an explicit geometric
        transformation). The fusion is done by taking the pixel-wise maximum
        of the masks from the left and right camera. If only one camera
        has produced a mask so far, that mask is published as-is.
        """
        lane_left = self.lane_mask_left
        lane_right = self.lane_mask_right
        da_left = self.driveable_area_left
        da_right = self.driveable_area_right

        # Nothing to publish yet
        if lane_left is None and lane_right is None:
            return

        if (
            lane_left is not None
            and lane_right is not None
            and da_left is not None
            and da_right is not None
        ):
            # Ensure both masks have the same resolution
            if lane_left.shape != lane_right.shape:
                h, w = lane_left.shape[:2]
                lane_right_resized = cv2.resize(
                    lane_right, (w, h), interpolation=cv2.INTER_NEAREST
                )
                da_right_resized = cv2.resize(
                    da_right, (w, h), interpolation=cv2.INTER_NEAREST
                )
            else:
                lane_right_resized = lane_right
                da_right_resized = da_right

            fused_lane = np.maximum(lane_left, lane_right_resized)
            fused_da = np.maximum(da_left, da_right_resized)
        elif lane_left is not None:
            fused_lane = lane_left
            fused_da = da_left
        else:
            fused_lane = lane_right
            fused_da = da_right

        ros_driveable_area = self.bridge.cv2_to_imgmsg(fused_da)
        ros_lane_mask = self.bridge.cv2_to_imgmsg(fused_lane)

        # publish fused masks on the existing Center topics
        self.lane_mask_publisher.publish(ros_lane_mask)
        self.driveable_area_publisher.publish(ros_driveable_area)

    def handle_dist_array(self, dist_array):
        """
        This function overwrites the current depth image from
        the lidar distance node with the latest depth image.

        Args:
            dist_array (image msg): Depth image from Lidar Distance Node
        """
        # callback function for lidar depth image
        # since frequency is lower than image frequency
        # the latest lidar image is saved
        dist_array = self.bridge.imgmsg_to_cv2(
            img_msg=dist_array, desired_encoding="passthrough"
        )
        self.dist_arrays = dist_array

    def preprocess_image(self, image):
        """
        Preprocesses the image to be fed into the model

        Args:
            image (ImgMsg): Image from camera

        Returns:
            np.array: Preprocessed image
        """

        cv_image = self.bridge.imgmsg_to_cv2(img_msg=image, desired_encoding="rgb8")
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        pil_image = Image.fromarray(cv_image)
        preprocess = t.Compose(
            [
                t.Resize((288, 800)),
                t.ToTensor(),
                t.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]),
            ]
        )
        input_image = preprocess(pil_image).unsqueeze(dim=0)
        return input_image, cv_image

    def detect_lanes(self, image):
        """
        Detects lanes in the image

        Args:
            image (np.array): preprocessed Image from camera

        Returns:
            np.array: Lane mask
        """
        det_out, da_seg_out, ll_seg_out = self.model(image)
        return det_out, da_seg_out, ll_seg_out

    def postprocess_image(self, da_seg_out, ll_seg_out):
        """resizes and binarizes the output of the model

        Args:
            da_seg_out (_type_): driveablearea
            ll_seg_out (_type_): lanemask

        Returns:
            postprocessed driveable area mask and lane mask
        """
        # convert probabilities to numpy array
        da_seg_out = da_seg_out.sigmoid().squeeze().cpu().numpy()  # (2, 288, 800)
        ll_seg_out = ll_seg_out.sigmoid().squeeze().cpu().numpy()  # (2, 288, 800)

        # scale output to original image
        da_seg_resized = cv2.resize(
            da_seg_out[1, :, :],
            (self.original_w, self.original_h),
            interpolation=cv2.INTER_LINEAR,
        )
        ll_seg_resized = cv2.resize(
            ll_seg_out[1, :, :],
            (self.original_w, self.original_h),
            interpolation=cv2.INTER_LINEAR,
        )

        # Dynamic threshold for binarization
        lane_threshold = np.mean(ll_seg_resized) + 0.07
        driveable_area_threshold = np.mean(da_seg_resized) + 0.07

        # Apply dynamic binarization
        ll_seg_binary = (ll_seg_resized > lane_threshold).astype(
            np.uint8
        )  # 1 for lane, 0 otherwise
        da_seg_binary = (da_seg_resized > driveable_area_threshold).astype(
            np.uint8
        )  # 1 for drivable area, 0 otherwise

        # Scale binary mask to match the visualization range [0, 255] for the overlay
        ll_seg_scaled = (ll_seg_binary * 255).astype(np.uint8)
        da_seg_scaled = (da_seg_binary * 255).astype(np.uint8)

        return ll_seg_scaled, da_seg_scaled


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Lanedetection_node()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

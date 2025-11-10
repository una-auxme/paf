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
            callback=self.image_handler,
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

    def image_handler(self, ImageMsg):
        """
        Callback function for image subscriber
        applies lane detection and Driveable area detection to given ImageMsg
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

        ros_driveable_area = self.bridge.cv2_to_imgmsg(da_seg_scaled)
        ros_lane_mask = self.bridge.cv2_to_imgmsg(ll_seg_scaled)

        # publish
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

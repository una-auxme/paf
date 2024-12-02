#!/usr/bin/env python

from ros_compatibility.node import CompatibleNode

from rospy.numpy_msg import numpy_msg

# import rospy

import numpy as np
import ros_compatibility as roscomp

import cv2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header

from cv_bridge import CvBridge

# for the lane detection model
import torch
from PIL import Image
import torchvision.transforms as t


class Lanedetection_node(CompatibleNode):
    """CLRerNet:
    Model for Lanedetection

    subscribes to camera image and publishes lane masks for further planning
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # load model
        self.model = torch.hub.load("hustvl/yolop", "yolop", pretrained=True)

        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.bridge = CvBridge()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        self.role_name = self.get_param("role_name", "hero")
        self.setup_camera_subscriptions("Center")
        self.setup_lane_publisher()
        self.setup_overlay_publisher()

    def run(self):
        self.spin()
        pass

    def setup_camera_subscriptions(self, side):
        """
        sets up a subscriber to the selected camera angle

        Args:
            side (String): Camera angle specified in launch file
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.image_handler,
            topic=f"/carla/{self.role_name}/{side}/image",
            qos_profile=1,
        )

    def setup_lane_publisher(self):
        """sets up a publisher for the lane mask
        topic: /Center/lane_img
        """
        self.lane_mask_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/lane_mask",
            qos_profile=1,
        )

    def setup_overlay_publisher(self):
        """sets up a publisher for the lane mask
        topic: /Center/lane_img
        """
        self.lane_overlay_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/Lane_detect_Overlay",
            qos_profile=1,
        )

    def image_handler(self, ImageMsg):
        """
        Callback function for image subscriber
        """
        # free up cuda memory
        if self.device == "cuda":
            torch.cuda.empty_cache()

        image, original_image = self.preprocess_image(ImageMsg)
        original_h, original_w, _ = original_image.shape
        with torch.no_grad():
            det_out, da_seg_out, ll_seg_out = self.detect_lanes(image)

        # convert probabilities to numpy array
        da_seg_out = da_seg_out.sigmoid().squeeze().cpu().numpy()  # (288, 800)
        ll_seg_out = ll_seg_out.sigmoid().squeeze().cpu().numpy()  # (2, 288, 800)

        # scale output to original image
        da_seg_resized = cv2.resize(
            da_seg_out[1, :, :],
            (original_w, original_h),
            interpolation=cv2.INTER_LINEAR,
        )
        ll_seg_resized = cv2.resize(
            ll_seg_out[1, :, :],
            (original_w, original_h),
            interpolation=cv2.INTER_LINEAR,
        )

        # extrace chanel
        ll_seg_out_lane = ll_seg_out[1, :, :]  

        # dynamic scaling of the array
        ll_seg_scaled = (
            (ll_seg_resized - np.min(ll_seg_resized))
            / (np.max(ll_seg_resized) - np.min(ll_seg_resized))
            * 255
        ).astype(np.uint8)

        # dynamic scaling of the array
        da_seg_scaled = (
            (da_seg_resized - np.min(da_seg_resized))
            / (np.max(da_seg_resized) - np.min(da_seg_resized))
            * 255
        ).astype(np.uint8)

        # convert original image to numpy
        overlay = np.array(original_image)

        # lane mask = blue
        overlay[:, :, 0] = cv2.addWeighted(overlay[:, :, 0], 1.0, ll_seg_scaled, 1, 0)

        # driveable area = green
        overlay[:, :, 1] = cv2.addWeighted(overlay[:, :, 1], 1.0, da_seg_scaled, 0.5, 0)

        ros_lane_mask = self.bridge.cv2_to_imgmsg(ll_seg_out_lane)
        ros_lane_overlay = self.bridge.cv2_to_imgmsg(overlay, encoding="bgr8")

        # ros_image.header = image.header
        # publish
        self.lane_mask_publisher.publish(ros_lane_mask)
        self.lane_overlay_publisher.publish(ros_lane_overlay)

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


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = Lanedetection_node("Lanedetection_node")
    node.run()
    print("Lanedetection_node started")

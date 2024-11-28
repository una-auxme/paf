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
        self.lane_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/lane_img",
            qos_profile=1,
        )

    def image_handler(self, ImageMsg):
        """
        Callback function for image subscriber
        """
        # free up cuda memory
        if self.device == "cuda":
            torch.cuda.empty_cache()

        image = self.preprocess_image(ImageMsg)

        det_out, da_seg_out, ll_seg_out = self.detect_lanes(image)

        ll_seg_out = ll_seg_out.detach().cpu().numpy()  # Detach and move to CPU
        # Ensure the tensor has the correct shape before transposing
        if ll_seg_out.ndim == 4:  # Batch x Channels x H x W
            ll_seg_out = ll_seg_out[0]  # Remove batch dimension
        np_ll_img = np.transpose(ll_seg_out, (2, 0, 1))
        ll_img = cv2.cvtColor(np_ll_img, cv2.COLOR_BGR2RGB)
        # publish vision result to rviz
        img_msg = self.bridge.cv2_to_imgmsg(ll_img, encoding="rgb8")
        img_msg.header = image.header
        # publish
        self.lane_publisher.publish(img_msg)

    def preprocess_image(self, image):
        """
        Preprocesses the image to be fed into the model

        Args:
            image (np.array): Image from camera

        Returns:
            np.array: Preprocessed image
        """
        """pil_image = Image.fromarray(image)

        preprocess = transforms.Compose(
            [
                transforms.Resize((288, 800)),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
                ),
            ]
        )
        return preprocess(pil_image).unsqueeze(0)
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
        return input_image

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

    def apply_mask(self, input_image, model_output):
        """function to draw the lane mask into the original image

        Args:
            input_image (np.array): original input image
            model_output (np.array): lane prediction
        """


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = Lanedetection_node("Lanedetection_node")
    node.run()
    print("Lanedetection_node started")

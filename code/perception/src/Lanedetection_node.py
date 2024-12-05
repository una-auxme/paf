#!/usr/bin/env python

import os
from ros_compatibility.node import CompatibleNode

from rospy.numpy_msg import numpy_msg

# import rospy

# import numpy as np
import ros_compatibility as roscomp

# import cv2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header

from cv_bridge import CvBridge

# for the lane detection model
# import torch

# from CLRerNet_model.configs.clrernet.culane import clrernet_culane_dla34_ema

# from CLRerNet_model import clrernet_culane_dla34

# from mmcv import Config
# from CLRerNet_model.libs.models.detectors import clrernet as build_detector
from mmdet.apis import init_detector

from libs.api.inference import inference_one_image
from libs.utils.visualizer import visualize_lanes

# for image preprocessing
from torchvision import transforms
from PIL import Image


class Lanedetection_node(CompatibleNode):
    """CLRerNet:
    Model for Lanedetection

    subscribes to camera image and publishes lane masks for further planning
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # WS path
        ws_path = os.path.dirname(os.path.realpath(__file__))
        ws_path = "/opt"
        print(ws_path)

        # Weights path
        weight_path = os.path.join("CLRerNet_model", "clrernet_culane_dla34_ema.pth")
        ws_weight_path = os.path.join(ws_path, weight_path)

        # Config path
        config_path = os.path.join(
            "CLRerNet_model/configs/clrernet/culane", "clrernet_culane_dla34_ema.py"
        )  # CLRerNet_model.configs.clrernet.culane
        ws_config_path = os.path.join(ws_path, config_path)

        # build the model from a config file and a checkpoint file
        model = init_detector(ws_config_path, ws_weight_path, device="cuda:0")
        print(model)

        # test a single image
        image_path = os.path.join("CLRerNet_model", "demo.jpg")
        image_path = "/opt/CLRerNet_model/demo/demo.jpg"
        image2_path = "/opt/CLRerNet_model/demo/result.jpg"
        image2_path = os.path.join("CLRerNet_model", "result.jpg")
        # ws_image_path = os.path.join(ws_path, image_path)
        # ws_image2_path = os.path.join(ws_path, image2_path)
        src, preds = inference_one_image(model, ws_image_path)
        # show the results
        # dst = visualize_lanes(src, preds, save_path=ws_image2_path)

        # ---------- OLD ------------
        # with open(ws_weight_path, "rb") as file:
        #    self.state_dict = torch.load(file)

        # self.model.load_state_dict(self.state_dict)

        # self.model.eval()
        # print("Model erforgreich geladen:", self.model, model_path)
        # ----------  ------------
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
        self.lane_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/lane_img",
            qos_profile=1,
        )

    def image_handler(self, ImageMsg):
        """
        Callback function for image subscriber
        """

        # image = self.bridge.imgmsg_to_cv2(ImageMsg, "bgr8")
        # image = self.preprocess_image(image)

        # lane_mask = self.detect_lanes(image, self.model)

        # self.lane_publisher.publish(ImageMsg)

    def preprocess_image(self, image):
        """
        Preprocesses the image to be fed into the model

        Args:
            image (np.array): Image from camera

        Returns:
            np.array: Preprocessed image
        ""
        pil_image = Image.fromarray(image)

        preprocess = transforms.Compose(
            [
                transforms.Resize((288, 800)),
                transforms.ToTensor(),
                transforms.Normalize(
                    mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225]
                ),
            ]
        )

        return preprocess(pil_image).unsqueeze(0).cuda()"""

    def detect_lanes(self, image, model):
        """
        Detects lanes in the image

        Args:
            image (np.array): Image from camera

        Returns:
            np.array: Lane mask
        """

    #    with torch.no_grad():
    #        output = model(image)

    #    return output


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = Lanedetection_node("Lanedetection_node")
    node.run()
    print("Lanedetection_node started")

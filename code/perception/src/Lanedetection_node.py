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
import torch
import cv2


# import matplotlib.pyplot as plt
from mmdet.apis import init_detector

# from libs.api.inference import inference_one_image
from libs.utils.visualizer import visualize_lanes
from libs.datasets.pipelines import Compose
from libs.datasets.metrics.culane_metric import interp
from mmcv.parallel import collate, scatter

# for image preprocessing
# from torchvision import transforms
# from PIL import Image


class Lanedetection_node(CompatibleNode):
    """CLRerNet:
    Model for Lanedetection

    subscribes to camera image and publishes lane masks for further planning
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        #
        self.img_freq = 20
        self.img_cnt = 0

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
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.model = init_detector(
            ws_config_path, ws_weight_path, device=self.device
        )  # "cuda:0")
        # print(self.model)

        # test a single image
        # image_path = "/opt/CLRerNet_model/demo/demo.jpg"

        # image_path = "/workspace/code/perception/src/ld_test4.jpg"
        # image2_path = "/workspace/code/perception/src/result6.jpg"

        # Bildgröße ermitteln
        # image = cv2.imread(image_path)
        # height, width = image.shape[:2]

        # Zielgröße definieren
        # target_width = 1640
        # target_height = 590

        # Berechne die Cropping-Koordinaten
        # x_min = (width - target_width) // 2
        # x_max = x_min + target_width
        # y_min = (height - target_height) // 2
        # y_max = y_min + target_height

        # Zuschneiden
        # cropped_image = image[y_min:y_max, x_min:x_max]

        # cv2.imshow("Cropped Image", cropped_image)
        # cv2.imwrite("/workspace/code/perception/src/cropped_image5.jpg", cropped_image)

        # croppedimage_path = "/workspace/code/perception/src/cropped_image5.jpg"

        # GPU-Speicher freigeben
        # torch.cuda.empty_cache()
        # src, preds = self.inference_one_image(self.model, croppedimage_path)
        # show the results
        # dst = visualize_lanes(src, preds, save_path=image2_path)

        # Display the result using Matplotlib
        # plt.figure(figsize=(10, 10))  # Optional: set the figure size
        # plt.imshow(dst[..., ::-1])  # Convert BGR (OpenCV default) to RGB for Matplotlib
        # plt.axis("off")  # Optional: turn off axis labels
        # plt.title("Lane Detection Result")  # Optional: add a title
        # plt.show()

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
        self.img_cnt += 1
        if self.img_cnt == self.img_freq:
            self.image = self.bridge.imgmsg_to_cv2(ImageMsg, "bgr8")
            # self.image = self.preprocess_image(self.image)

            lane_mask = self.detect_lanes(self.image, self.model)

            # plt.figure(figsize=(10, 10))  # Optional: set the figure size
            # plt.imshow(
            #    lane_mask[..., ::-1]
            # )  # Convert BGR (OpenCV default) to RGB for Matplotlib
            # plt.axis("off")  # Optional: turn off axis labels
            # plt.title("Lane Detection Result")  # Optional: add a title
            # plt.show()
            lanedetection_image = self.bridge.cv2_to_imgmsg(lane_mask, "bgr8")
            self.lane_publisher.publish(lanedetection_image)
            self.img_cnt = 0

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

        height, width = image.shape[:2]

        # Zielgröße definieren
        target_width = 1640
        target_height = 590

        # Berechne die Cropping-Koordinaten
        if width > target_width:
            x_min = (width - target_width) // 2
            x_max = x_min + target_width
        else:
            x_min = (target_width - width) // 2
            x_max = x_min + width
        if height > target_height:
            y_min = (height - target_height) // 2
            y_max = y_min + target_height
        else:
            y_min = (target_height - height) // 2
            y_max = y_min + height
        # Zuschneiden
        cropped_image = image[y_min:y_max, x_min:x_max]

        return cropped_image

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
        # image = cv2.imdecode(image, cv2.IMREAD_COLOR)
        height, width, channels = image.shape
        print(f"Breite: {width}px")
        print(f"Höhe: {height}px")
        image_resized = self.resize_image(image, 1640, 590)
        # image = cv2.resize(image, (180, 590))

        # plt.figure(figsize=(10, 10))  # Optional: set the figure size
        # plt.imshow(image_resized)  # Convert BGR (OpenCV default) to RGB for Matplotlib
        # plt.axis("off")  # Optional: turn off axis labels
        # plt.title("Lane Detection Result")  # Optional: add a title
        # plt.show()

        torch.cuda.empty_cache()
        src, preds = self.inference_one_image(model, image_resized)
        # show the results
        dst = visualize_lanes(src, preds)

        return dst

    def inference_one_image(self, model, image):
        """Inference on an image with the detector.
        Args:
            model (nn.Module): The loaded detector.
            img_path (str): Image path.
        Returns:
            img (np.ndarray): Image data with shape (width, height, channel).
            preds (List[np.ndarray]): Detected lanes.
        """
        # img = cv2.imread(image)
        ori_shape = image.shape
        data = dict(
            filename="img",
            sub_img_name=None,
            img=image,
            gt_points=[],
            id_classes=[],
            id_instances=[],
            img_shape=ori_shape,
            ori_shape=ori_shape,
        )

        cfg = model.cfg
        model.bbox_head.test_cfg.as_lanes = False
        device = next(model.parameters()).device  # model device

        test_pipeline = Compose(cfg.data.test.pipeline)

        data = test_pipeline(data)
        data = collate([data], samples_per_gpu=1)

        data["img_metas"] = data["img_metas"].data[0]
        data["img"] = data["img"].data[0]

        if next(model.parameters()).is_cuda:
            # scatter to specified GPU
            data = scatter(data, [device])[0]

        # forward the model
        with torch.no_grad():
            results = model(return_loss=False, rescale=True, **data)

        lanes = results[0]["result"]["lanes"]
        preds = self.get_prediction(lanes, ori_shape[0], ori_shape[1])

        return image, preds

    def get_prediction(self, lanes, ori_h, ori_w):
        preds = []
        for lane in lanes:
            lane = lane.cpu().numpy()
            xs = lane[:, 0]
            ys = lane[:, 1]
            valid_mask = (xs >= 0) & (xs < 1)
            xs = xs * ori_w
            lane_xs = xs[valid_mask]
            lane_ys = ys[valid_mask] * ori_h
            lane_xs, lane_ys = lane_xs[::-1], lane_ys[::-1]
            pred = [(x, y) for x, y in zip(lane_xs, lane_ys)]
            interp_pred = interp(pred, n=5)
            preds.append(interp_pred)
        return preds

    def resize_image(self, image, target_width, target_height):
        return cv2.resize(image, (target_width, target_height))


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = Lanedetection_node("Lanedetection_node")
    node.run()
    print("Lanedetection_node started")

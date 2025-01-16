#!/usr/bin/env python

import os
from ros_compatibility.node import CompatibleNode

from rospy.numpy_msg import numpy_msg
import ros_compatibility as roscomp

# import cv2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header
from cv_bridge import CvBridge

# for the lane detection model
import torch
import cv2
import time
import subprocess
import matplotlib.pyplot as plt
from mmdet.apis import init_detector
import numpy as np

# from libs.api.inference import inference_one_image
from libs.utils.visualizer import visualize_lanes
from libs.datasets.pipelines import Compose
from libs.datasets.metrics.culane_metric import interp
from mmcv.parallel import collate, scatter


class Lanedetection_node(CompatibleNode):
    """CLRerNet:
    Model for Lanedetection

    subscribes to camera image and publishes lane masks for further planning
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        # Frequence of using Lanedetection model on incoming Images
        self.img_freq = 10
        self.img_cnt = 0

        # WS path
        ws_path = os.path.dirname(os.path.realpath(__file__))
        ws_path = "/opt"
        print(ws_path)

        # Weights path
        weight_path = os.path.join("CLRerNet_model", "clrernet_culane_dla34_ema.pth")
        ws_weight_path = os.path.join(ws_path, weight_path)
        # weight_path = '/workspace/opt'

        # Config path
        config_path = os.path.join(
            "CLRerNet_model/configs/clrernet/culane", "clrernet_culane_dla34_ema.py"
        )  # CLRerNet_model.configs.clrernet.culane
        ws_config_path = os.path.join(ws_path, config_path)

        torch.cuda.empty_cache()
        torch.zeros(1).to("cuda")

        self.wait_for_gpu(min_free_memory_mb=1000)

        # build the model from a config file and a checkpoint file
        # self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        # -> Model cannot use cpu
        self.model = init_detector(
            ws_config_path, ws_weight_path, device="cuda:0"
        )  # self.device

        self.wait_for_gpu(min_free_memory_mb=1000)

        torch.cuda.empty_cache()
        torch.cuda.synchronize()

        self.bridge = CvBridge()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        self.role_name = self.get_param("role_name", "hero")
        self.setup_camera_subscriptions("Center")
        self.setup_lane_publisher()

    def run(self):
        self.spin()
        pass

    def test_single_img(self, image_path, result_image_path):
        """
        Function to test with a single dummy picture

        Reads Image from path
        Resizes Image
        Uses Lanedetection
        Saves and plots finished image
        """

        # image_path = "/workspace/code/perception/src/ld_test4.jpg"
        # image2_path = "/workspace/code/perception/src/result.jpg"

        # Bildgröße ermitteln
        image = cv2.imread(image_path)
        height, width = image.shape[:2]

        # Zielgröße definieren
        target_width = 1640
        target_height = 590

        # Berechne die Cropping-Koordinaten
        x_min = (width - target_width) // 2
        x_max = x_min + target_width
        y_min = (height - target_height) // 2
        y_max = y_min + target_height

        # Zuschneiden
        cropped_image = image[y_min:y_max, x_min:x_max]

        cv2.imshow("Cropped Image", cropped_image)
        cv2.imwrite("/workspace/code/perception/src/cropped_image5.jpg", cropped_image)

        croppedimage_path = "/workspace/code/perception/src/cropped_image5.jpg"

        # GPU-Speicher freigeben
        torch.cuda.empty_cache()
        src, preds = self.inference_one_image(self.model, croppedimage_path)
        # show the results
        dst = visualize_lanes(src, preds, save_path=result_image_path)

        # Display the result using Matplotlib
        plt.figure(figsize=(10, 10))  # Optional: set the figure size
        plt.imshow(dst[..., ::-1])  # Convert BGR (OpenCV default) to RGB for Matplotlib
        plt.axis("off")  # Optional: turn off axis labels
        plt.title("Lane Detection Result")  # Optional: add a title
        plt.show()

    def wait_for_gpu(self, min_free_memory_mb=500, check_interval=1):
        """
        Waits until `min_free_memory_mb` is available on GPU
        :param min_free_memory_mb: Minimum space in MB
        :param check_interval: Waiting interval for space-check (in sec)
        """
        if not torch.cuda.is_available():
            print("No GPU available. Exiting wait function.")
            return

        while True:
            used_memory, free_memory, total_memory = self.get_gpu_memory_nvidia_smi()
            if used_memory is None or free_memory is None or total_memory is None:
                print("Unable to retrieve GPU memory information. Exiting.")
                return

            print(
                f"GPU Memory Check - Used: {used_memory} MB"
                f"GPU Memory Check - Used: Free: {free_memory} MB"
                f"GPU Memory Check - Used: Total: {total_memory} MB"
            )

            if free_memory >= min_free_memory_mb:
                print(f"Enough free memory available ({free_memory} MB). Proceeding...")
                break
            else:
                print("Waiting for GPU to free up memory...")
                time.sleep(check_interval)

    def get_gpu_memory_nvidia_smi(self):
        """
        Calls GPU Space-information `nvidia-smi`
        Returns (used_memory, free_memory, total_memory) in MB
        """
        try:
            # Use nvidia-smi and parse the output
            result = subprocess.run(
                [
                    "nvidia-smi",
                    "--query-gpu=memory.used,memory.free,memory.total",
                    "--format=csv,noheader,nounits",
                ],
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            # Processes the output of nvidia-smi
            output = result.stdout.strip()
            used_memory, free_memory, total_memory = map(
                int, output.split("\n")[0].split(", ")
            )
            return used_memory, free_memory, total_memory
        except Exception as e:
            print(f"Error retrieving GPU memory info: {e}")
            return None, None, None

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
        Uses counter `img_cnt` and `img_freq` to only use a certain percentage
        of images for lanedetction to minimize gpu-usage
        """
        self.img_cnt += 1
        if self.img_cnt >= self.img_freq:
            self.image = self.bridge.imgmsg_to_cv2(ImageMsg, "bgr8")
            # self.image = self.preprocess_image(self.image)

            lane_mask = self.detect_lanes(self.image, self.model)

            if lane_mask is not None:
                lanedetection_image = self.bridge.cv2_to_imgmsg(lane_mask, "bgr8")
                self.lane_publisher.publish(lanedetection_image)
            else:
                # Only there to see if node is running, if lane_mask returns null
                # -> model not working yet
                no_img = np.zeros_like(
                    self.image
                )  # Creates an empty image with the same dimensions
                lanedetection_image = self.bridge.cv2_to_imgmsg(no_img, "bgr8")
                self.lane_publisher.publish(lanedetection_image)

            self.img_cnt = 0

    def preprocess_image(self, image):
        """
        -- Not needed anymore --

        Preprocesses the image to be fed into the model

        """

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
        Resizes image for model
        Tries to use CLRerNet Model, if it fails the img_freq becomes bigger
        to give it more time

        Args:
            image (np.array): Image from camera

        Returns:
            np.array: Lane mask
        """
        # For debugging --
        # height, width, channels = image.shape
        # print(f"Breite: {width}px")
        # print(f"Höhe: {height}px")

        # Resize Image to needed size
        image_resized = cv2.resize(image, (1640, 590))

        # torch.cuda.empty_cache()
        try:
            src, preds = self.inference_one_image(model, image_resized)
            dst = visualize_lanes(src, preds)
            self.img_freq = 0
            print("Detect Lanes successfull")
            return dst
        except RuntimeError as e:
            self.img_freq += 5
            print("Detect Lanes unsuccessful")
            print(f"An exception occurred: {e}")
            torch.cuda.empty_cache()
            return None

    def inference_one_image(self, model, image):
        """Inference on an image with the detector.
        Args:
            model (nn.Module): The loaded detector.
            img_path (str): Image path.
        Returns:
            img (np.ndarray): Image data with shape (width, height, channel).
            preds (List[np.ndarray]): Detected lanes.
        """
        # Uses image instead of image_path
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


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    print("Lanedetection_node started")

    try:
        node = Lanedetection_node("Lanedetection_node")
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        roscomp.shutdown()

#!/usr/bin/env python3
"""
import pathlib
import numpy as np
import ros_compatibility as roscomp
import torch
import cv2
import albumentations as A
from panopticapi.utils import id2rgb
from rospy.numpy_msg import numpy_msg
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image

from panoptic_segmentation.efficientps import EfficientPS as EfficientPS
from panoptic_segmentation.train_net import add_custom_param
from panoptic_segmentation.efficientps.panoptic_segmentation_module
import panoptic_segmentation_module

from detectron2.config import get_cfg
import torchvision.transforms.functional as F
from detectron2.structures import Instances, BitMasks, Boxes

CFG_FILE_PATH = pathlib.Path(__file__).parent /
"panoptic_segmentation" / "config.yaml"

MODEL_PATH = pathlib.Path(__file__).parent.parent /
"src/panoptic_segmentation/efficientps/model.pth"


class SegmentationNode(CompatibleNode):

    This node runs the panoptic segmentation model on
    the camera images and publishes the segmented results.


    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.publisher = None
        self.loginfo("Initializing panoptic segmentation node...")

        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")

        self.model, self.transform, self.model_cfg = self.load_model()
        # warm up
        #self.predict(np.zeros((720, 1280, 3)))

        #self.setup_camera_subscriptions()
        #self.setup_camera_publishers()

    def setup_camera_subscriptions(self):

        self.new_subscription(
            msg_type=numpy_msg(Image),
            callback=self.handle_camera_image,
            topic=f"/carla/{self.role_name}/{self.side}/image",
            qos_profile=1
        )

    def setup_camera_publishers(self):

        self.publisher = self.new_publisher(
            msg_type=numpy_msg(Image),
            topic=f"/paf/{self.role_name}/{self.side}/segmented_image",
            qos_profile=1
        )

    @staticmethod
    def load_model():

        cfg = get_cfg()
        cfg['train'] = False
        add_custom_param(cfg)
        cfg.merge_from_file(str(CFG_FILE_PATH))

        transform = A.Compose([
            A.Resize(height=512, width=1024),
            A.Normalize(mean=cfg.TRANSFORM.NORMALIZE.MEAN,
                        std=cfg.TRANSFORM.NORMALIZE.STD),
        ])
        "model = EfficientPS.load_from_checkpoint(
            cfg=cfg,
            checkpoint_path=str(MODEL_PATH)
        )
        model = EfficientPS(cfg)
        model.load_state_dict(torch.load(MODEL_PATH))
        print(model)
        model.eval()
        model.freeze()
        model.to(torch.device("cuda:0")) #add device definition before

        return model, transform, None

    def predict(self, image: np.ndarray):

        self.loginfo(f"predicting image shape: {image.shape}")
        # expand
        # image = np.expand_dims(image, axis=0)
        transformed = self.transform(image=image)

        image = transformed["image"]
        instances = Instances(image_size=(image.shape[1], image.shape[0]))

        instances.gt_masks = BitMasks(torch.Tensor([]).view(0, 1, 1))
        instances.gt_classes = torch.as_tensor([])
        instances.gt_boxes = Boxes(torch.Tensor([]))
        inputs = {
            'image': torch.stack([F.to_tensor(image)]).to(
                torch.device("cuda:0")),
            'instance': [instances]
        }

        predictions = self.model(inputs)
        segmented_result = panoptic_segmentation_module(self.model_cfg,
                                                        predictions,
                                                        device=torch.device(
                                                            "cuda:0"))

        result = segmented_result.cpu().numpy()
        # self.loginfo(f"predictions: {prediction.shape}")
        return resultskip python linting
code/perception/src/p_testing_node.py:17:27: W291 trailing w
        self.loginfo("predicting something")

    def handle_camera_image(self, image):
        self.loginfo(f"got image from camera {self.side}")
        image_array = np.frombuffer(image.data, dtype=np.uint8)
        image_array = image_array.reshape((image.height, image.width, -1))
        # remove alpha channel
        image_array = image_array[:, :, :3]

        # image = image_array.reshape((image.height, image.width, 3))

        # self.loginfo(f"image shape: {image.data.shape}")

        # returns a numpy array of shape (512, 1024, 3)
        prediction = self.predict(image_array).squeeze()
        prediction = id2rgb(prediction)
        print(prediction.shape)
        # resize to (720, 1280, 3)
        img = cv2.resize(prediction, (1280, 720),
                         interpolation=cv2.INTER_NEAREST)
        output = img.tobytes()

        # construct the message
        msg = Image()
        msg.header.stamp = roscomp.ros_timestamp(
            self.get_time(), from_sec=True)
        msg.header.frame_id = "map"
        msg.height = 720
        msg.width = 1280
        msg.encoding = "rgb8"
        msg.is_bigendian = 0
        msg.step = 1280 * 3
        msg.data = output

        self.publisher.publish(msg)
        self.loginfo(f"prediction shape: {prediction.shape}")
        self.loginfo("reveived image from camera")
        pass

    def run(self):
        self.spin()
        pass
        # while True:
        #    self.spin()


if __name__ == "__main__":
    roscomp.init("SegmentationNode")
    # try:

    node = SegmentationNode("SegmentationNode")
    node.run()
    # except KeyboardInterrupt:
    #    pass
    # finally:
#       roscomp.shutdown()
#

"""

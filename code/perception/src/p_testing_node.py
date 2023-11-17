#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import torch
from torchvision.models.segmentation import deeplabv3_resnet101
from torchvision.models.segmentation import DeepLabV3_ResNet101_Weights
import torchvision.transforms as t
import cv2
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class PerceptionTestingNode(CompatibleNode):
    def __init__(self, name, **kwargs):
        # starting comment

        super().__init__(name, **kwargs)
        # self.model = torch.hub.load('pytorch/vision:v0.10.0',
        # 'deeplabv3_resnet50', pretrained=True)

        self.model = deeplabv3_resnet101(DeepLabV3_ResNet101_Weights)
        # self.model.eval()
        # print("Model Test: ", self.model(torch.zeros((1,3,720,1280))))

        self.bridge = CvBridge()

        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.setup_camera_subscriptions()
        self.setup_camera_publishers()

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

    def handle_camera_image(self, image):
        self.model.eval()
        self.loginfo(f"got image from camera {self.side}")

        cv_image = self.bridge.imgmsg_to_cv2(img_msg=image,
                                             desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        """
        image_array = np.frombuffer(image.data, dtype=np.uint8)
        print(image_array.shape)
        image_array = image_array.reshape((image.height, image.width, -1))
        print(image_array.shape)
        # remove alpha channel
        image_array = image_array[:, :, :3]
        print(image_array.shape)"""

        preprocess = t.Compose([
            t.ToTensor(),
            t.Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225])
        ])
        input_image = preprocess(cv_image).unsqueeze(dim=0)
        prediction = self.model(input_image)['out'][0]
        # prediction = id2rgb(prediction)
        # print(prediction)
        print(prediction.shape)

        masked_image = self.create_mask(prediction, input_image)
        self.publisher.publish(self.bridge.cv2_to_imgmsg(masked_image))

        pass

    def create_mask(self, model_output, input_image):
        palette = torch.tensor([2 ** 25 - 1, 2 ** 15 - 1, 2 ** 21 - 1])
        colors = torch.as_tensor([i for i in range(21)])[:, None] * palette
        colors = (colors % 255).numpy().astype("uint8")
        r = Image.fromarray(model_output.byte().cpu().numpy())
        r = r.resize(input_image.shape[2], input_image.shape[3])
        r.putpalette(colors)
        return r

    def run(self):
        self.spin()
        pass
        # while True:
        #    self.spin()


if __name__ == "__main__":
    roscomp.init("PerceptionTestingNode")
    # try:

    node = PerceptionTestingNode("PerceptionTestingNode")
    node.run()

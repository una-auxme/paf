#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import torch
from torchvision.models.segmentation import DeepLabV3_ResNet101_Weights,\
                                            deeplabv3_resnet101
from torchvision.models.detection.faster_rcnn import FasterRCNN_MobileNet_V3_Large_320_FPN_Weights,\
                                                     FasterRCNN_ResNet50_FPN_V2_Weights,\
                                                     fasterrcnn_resnet50_fpn_v2,\
                                                     fasterrcnn_mobilenet_v3_large_320_fpn
import torchvision.transforms as t
import torchvision
import cv2
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from sensor_msgs.msg import CameraInfo 
from std_msgs.msg import Header
from cv_bridge import CvBridge
from PIL import Image
from torchvision.utils import draw_bounding_boxes, draw_segmentation_masks
import numpy as np
from time import perf_counter
"""
VisionNode:

The vision node provides a base node for object-detection or image-segementation.
This node provides the following features:

- Insert pretrained AI-Models
- Subscription to one camera
- Preprocessing of Input Image
- Publishing output image


Models to insert:

- DeepLabV3_ResNet50:
    self.model = torch.hub.load('pytorch/vision:v0.10.0', 'deeplabv3_resnet50', pretrained=True)
- DeepLabV3_ResNet101:
    self.model = deeplabv3_resnet101(DeepLabV3_ResNet101_Weights)
    
"""


class VisionNode(CompatibleNode):
    def __init__(self, name, **kwargs):
        # vision node
        super().__init__(name, **kwargs)
        self.model_dict = {
            "fasterrcnn_resnet50_fpn_v2": (fasterrcnn_resnet50_fpn_v2(weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT), FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT, "detection", "pyTorch"),
            "fasterrcnn_mobilenet_v3_large_320_fpn": (fasterrcnn_mobilenet_v3_large_320_fpn(weights=FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT), FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT, "detection", "pyTorch"),
            "deeplabv3_resnet101": (deeplabv3_resnet101(weights=DeepLabV3_ResNet101_Weights.DEFAULT), DeepLabV3_ResNet101_Weights.DEFAULT, "segmentation", "pyTorch")
        }
        

        #general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        #self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu") Cuda Memory Issues
        self.device = torch.device("cpu")
        print("VisionNode working on: ", self.device)

        #publish / subscribe setup
        self.setup_camera_subscriptions()
        self.setup_camera_publishers()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        #model setup
        model_info = self.model_dict[self.get_param("model")]
        self.model = model_info[0]
        self.weights = model_info[1]
        self.type = model_info[2]
        self.framework = model_info[3]
        print(f"Vision Node Configuration: Model -> {self.get_param('model')}, Type -> {self.type}, Framework -> {self.framework}")
        self.model.to(self.device)

    def setup_camera_subscriptions(self):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic=f"/carla/{self.role_name}/{self.side}/image",
            qos_profile=1
        )


    def setup_camera_publishers(self):
        self.publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/{self.side}/segmented_image",
            qos_profile=1
        )
    

    def handle_camera_image(self, image):
        startTime = perf_counter()
        self.model.eval()
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=image,
                                             desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        preprocess = t.Compose([
            t.ToTensor(),
            t.Normalize(mean=[0.485, 0.456, 0.406],
                        std=[0.229, 0.224, 0.225])
        ])
        
        input_image = preprocess(cv_image).unsqueeze(dim=0)
        input_image = input_image.to(self.device)
        print("Before Model: ", perf_counter() - startTime)
        prediction = self.model(input_image)
        print("After Model: ", perf_counter() - startTime)
        if(self.type == "detection"):
            vision_result = self.apply_bounding_boxes(cv_image, prediction[0])
        if(self.type == "segmentation"):
            vision_result = self.create_mask(cv_image, prediction['out'])

        img_msg = self.bridge.cv2_to_imgmsg(vision_result, encoding="passthrough")
        img_msg.header = image.header
       
        self.publisher.publish(img_msg)
        print("After Publish: ", perf_counter() - startTime)
        
        pass

    def create_mask(self, input_image, model_output):
        output_predictions = torch.argmax(model_output, dim=0)

        for i in range(21):
            output_predictions[i] = output_predictions[i] == i

        output_predictions = output_predictions.to(dtype=torch.bool)
        
        input_image = t.ToTensor()(input_image)
        input_image = input_image.to(dtype=torch.uint8)
        print(output_predictions.shape)
        print(input_image.shape)
        segmented_image = draw_segmentation_masks(input_image, output_predictions)
        cv_segmented = cv2.cvtColor(segmented_image.detach().numpy(), cv2.COLOR_BGR2RGB)
        return cv_segmented

    def apply_bounding_boxes(self, input_image, model_output):
        transposed_image = np.transpose(input_image, (2, 0, 1))
        image_np_with_detections = torch.tensor(transposed_image, dtype=torch.uint8)
        boxes = model_output['boxes']
        #scores = model_output['scores']
        labels = [self.weights.meta["categories"][i] for i in model_output['labels']]

        box = draw_bounding_boxes(image_np_with_detections, boxes, labels, colors='red', width=2)
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

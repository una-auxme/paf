#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
import torch
from torchvision.models.segmentation import DeepLabV3_ResNet101_Weights, \
                                            deeplabv3_resnet101
from torchvision.models.detection.faster_rcnn import \
       FasterRCNN_MobileNet_V3_Large_320_FPN_Weights, \
       FasterRCNN_ResNet50_FPN_V2_Weights, \
       fasterrcnn_resnet50_fpn_v2, \
       fasterrcnn_mobilenet_v3_large_320_fpn
import torchvision.transforms as t
import cv2
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header, Float32MultiArray
from cv_bridge import CvBridge
from torchvision.utils import draw_bounding_boxes, draw_segmentation_masks
import numpy as np
from ultralytics import NAS, YOLO, RTDETR, SAM, FastSAM
# import rospy

"""
VisionNode:

The vision node provides a base node for object-detection
or image-segementation.
This node provides the following features:

- Insert pretrained AI-Models
- Subscription to one camera
- Preprocessing of Input Image
- Publishing output image
"""


class VisionNode(CompatibleNode):
    def __init__(self, name, **kwargs):
        # vision node
        super().__init__(name, **kwargs)
        self.model_dict = {
            "fasterrcnn_resnet50_fpn_v2":
            (fasterrcnn_resnet50_fpn_v2(
                weights=FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT),
                FasterRCNN_ResNet50_FPN_V2_Weights.DEFAULT,
                "detection",
                "pyTorch"),
            "fasterrcnn_mobilenet_v3_large_320_fpn":
            (fasterrcnn_mobilenet_v3_large_320_fpn(
                weights=FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT),
                FasterRCNN_MobileNet_V3_Large_320_FPN_Weights.DEFAULT,
                "detection",
                "pyTorch"),
            "deeplabv3_resnet101":
            (deeplabv3_resnet101(
                weights=DeepLabV3_ResNet101_Weights.DEFAULT),
                DeepLabV3_ResNet101_Weights.DEFAULT,
                "segmentation",
                "pyTorch"),
            'yolov8n': (YOLO, "yolov8n.pt", "detection", "ultralytics"),
            'yolov8s': (YOLO, "yolov8s.pt", "detection", "ultralytics"),
            'yolov8m': (YOLO, "yolov8m.pt", "detection", "ultralytics"),
            'yolov8l': (YOLO, "yolov8l.pt", "detection", "ultralytics"),
            'yolov8x': (YOLO, "yolov8x.pt", "detection", "ultralytics"),
            'yolo_nas_l': (NAS, "yolo_nas_l.pt", "detection", "ultralytics"),
            'yolo_nas_m': (NAS, "yolo_nas_m.pt", "detection", "ultralytics"),
            'yolo_nas_s': (NAS, "yolo_nas_s.pt", "detection", "ultralytics"),
            'rtdetr-l': (RTDETR, "rtdetr-l.pt", "detection", "ultralytics"),
            'rtdetr-x': (RTDETR, "rtdetr-x.pt", "detection", "ultralytics"),
            'yolov8x-seg': (YOLO, "yolov8x-seg.pt", "segmentation",
                            "ultralytics"),
            'sam_l': (SAM, "sam_l.pt", "detection", "ultralytics"),
            'FastSAM-x': (FastSAM, "FastSAM-x.pt", "detection", "ultralytics"),

        }

        print(torch.__version__)

        # general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.center = self.get_param("center")
        self.back = self.get_param("back")
        self.left = self.get_param("left")
        self.right = self.get_param("right")

        self.device = torch.device("cuda"
                                   if torch.cuda.is_available() else "cpu")
        self.depth_images = []
        self.dist_arrays = None

        # publish / subscribe setup
        if self.center:
            self.setup_camera_subscriptions("Center")
        if self.back:
            self.setup_camera_subscriptions("Back")
        if self.left:
            self.setup_camera_subscriptions("Left")
        if self.right:
            self.setup_camera_subscriptions("Right")

        # self.setup_rainbow_subscription()
        self.setup_dist_array_subscription()
        self.setup_camera_publishers()
        self.setup_object_distance_publishers()
        self.setup_traffic_light_publishers()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        # model setup
        model_info = self.model_dict[self.get_param("model")]
        self.model = model_info[0]
        self.weights = model_info[1]
        self.type = model_info[2]
        self.framework = model_info[3]
        self.save = True
        print("Vision Node Configuration:")
        print("Device -> ", self.device)
        print(f"Model -> {self.get_param('model')},")
        print(f"Type -> {self.type}, Framework -> {self.framework}")
        torch.cuda.memory.set_per_process_memory_fraction(0.1)

        # pyTorch and CUDA setup
        if self.framework == "pyTorch":
            for param in self.model.parameters():
                param.requires_grad = False
                self.model.to(self.device)

        # ultralytics setup
        if self.framework == "ultralytics":
            self.model = self.model(self.weights)

        # tensorflow setup

    def setup_camera_subscriptions(self, side):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic=f"/carla/{self.role_name}/{side}/image",
            qos_profile=1
        )
        # print(f"Subscribed to Side: {side}")

    def setup_rainbow_subscription(self):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_rainbow_image,
            topic='/paf/hero/Center/rainbow_image',
            qos_profile=1
        )

    def setup_dist_array_subscription(self):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_dist_array,
            topic='/paf/hero/Center/dist_array',
            qos_profile=1
        )

    def setup_camera_publishers(self):
        if self.center:
            self.publisher_center = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Center/segmented_image",
                qos_profile=1
            )
            # print("Publisher to Center!")
        if self.back:
            self.publisher_back = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Back/segmented_image",
                qos_profile=1
            )
            # print("Publisher to Back!")
        if self.left:
            self.publisher_left = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Left/segmented_image",
                qos_profile=1
            )
            # print("Publisher to Left!")
        if self.right:
            self.publisher_right = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Right/segmented_image",
                qos_profile=1
            )
            # print("Publisher to Right!")

    def setup_object_distance_publishers(self):
        self.distance_publisher = self.new_publisher(
            msg_type=numpy_msg(Float32MultiArray),
            topic=f"/paf/{self.role_name}/{self.side}/object_distance",
            qos_profile=1)

    def setup_traffic_light_publishers(self):
        self.traffic_light_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/{self.side}/segmented_traffic_light",
            qos_profile=1
        )

    def handle_camera_image(self, image):
        # free up cuda memory
        if self.device == "cuda":
            torch.cuda.empty_cache()

        if self.framework == "pyTorch":
            vision_result = self.predict_torch(image)

        if self.framework == "ultralytics":
            vision_result = self.predict_ultralytics(image)

        # publish image to rviz
        """img_msg = self.bridge.cv2_to_imgmsg(vision_result,
                                            encoding="rgb8")
        img_msg.header = image.header
        side = rospy.resolve_name(img_msg.header.frame_id).split('/')[2]
        if side == "Center":
            self.publisher_center.publish(img_msg)
        if side == "Back":
            self.publisher_back.publish(img_msg)
        if side == "Left":
            self.publisher_left.publish(img_msg)
        if side == "Right":
            self.publisher_right.publish(img_msg)"""

        locals().clear()
        # print(f"Published Image on Side: {side}")
        pass

    def handle_rainbow_image(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=image,
                                             desired_encoding='passthrough')
        # cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)
        self.depth_images.append(cv_image)
        if len(self.depth_images) > 1:
            self.depth_images.pop(0)
            """if self.save:
                for i in range(len(self.depth_images)):
                    cv2.imshow(f"{i}", self.depth_images[i])

                self.save = False
                cv2.waitKey(0)  # Wait for any key press
                cv2.destroyAllWindows()"""

        else:
            self.logerr("Depth-Fiel build up! No distances available yet!")

    def handle_dist_array(self, dist_array):
        dist_array = \
            self.bridge.imgmsg_to_cv2(img_msg=dist_array,
                                      desired_encoding='passthrough')
        # print("RECEIVED DIST")
        self.dist_arrays = dist_array
        locals().clear()

    def predict_torch(self, image):
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
        prediction = self.model(input_image)

        if (self.type == "detection"):
            vision_result = self.apply_bounding_boxes(cv_image, prediction[0])
        if (self.type == "segmentation"):
            vision_result = self.create_mask(cv_image, prediction['out'])

        return vision_result

    def predict_ultralytics(self, image):
        cv_image = self.bridge.imgmsg_to_cv2(img_msg=image,
                                             desired_encoding='passthrough')
        cv_image = cv2.cvtColor(cv_image, cv2.COLOR_RGB2BGR)

        output = self.model(cv_image, half=True, verbose=False)

        # handle distance of objects
        distance_output = []
        c_boxes = []
        c_labels = []
        for r in output:
            boxes = r.boxes
            for box in boxes:
                cls = box.cls.item()
                pixels = box.xyxy[0]
                if self.dist_arrays is not None:
                    distances = np.asarray(
                        self.dist_arrays[int(pixels[1]):int(pixels[3]):1,
                                         int(pixels[0]):int(pixels[2]):1,
                                         ::])
                    condition = distances[:, :, 0] != 0
                    non_zero_filter = distances[condition]
                    distances_copy = distances.copy()
                    distances_copy[distances_copy == 0] = np.inf

                    if len(non_zero_filter) > 0:
                        sorted_indices = np.argsort(distances_copy[:, :, 0],
                                                    axis=None)
                        x1, y1 = np.unravel_index(sorted_indices[0],
                                                  distances_copy.shape[:2])
                        x2, y2 = np.unravel_index(sorted_indices[1],
                                                  distances_copy.shape[:2])
                        obj_dist1 = distances_copy[x1][y1].copy()
                        obj_dist2 = distances_copy[x2][y2].copy()

                        abs_distance = np.sqrt(
                            obj_dist1[0]**2 +
                            obj_dist1[1]**2 +
                            obj_dist1[2]**2)

                        # create 2d glass plane at object
                        # with box dimension
                        width_diff = abs(y1-y2)
                        height_diff = abs(x1-x2)

                        if width_diff > 0 and height_diff > 0:
                            scale_width = abs(obj_dist1[1] - obj_dist2[1])\
                                / width_diff
                            scale_height = abs(obj_dist1[2] - obj_dist2[2])\
                                / height_diff
                            width = distances_copy.shape[1] * scale_width
                            height = distances_copy.shape[0] * scale_height

                            # upper left
                            ul_x = obj_dist1[0]
                            ul_y = obj_dist1[1] - (-y1 + scale_width)
                            ul_z = obj_dist1[2] - (-x1 + scale_height)

                            # lower right
                            lr_x = obj_dist1[0]
                            lr_y = ul_y + width
                            lr_z = ul_z + height

                            distance_output.append([cls,
                                                    abs_distance,
                                                    ul_x, ul_y, ul_z,
                                                    lr_x, lr_y, lr_z])
                        else:
                            distance_output.append([cls,
                                                    abs_distance,
                                                    np.inf, np.inf, np.inf,
                                                    np.inf, np.inf, np.inf])

                    else:
                        obj_dist1 = (np.inf, np.inf, np.inf)
                        abs_distance = np.inf

                        distance_output.append([cls,
                                                abs_distance,
                                                np.inf, np.inf, np.inf,
                                                np.inf, np.inf, np.inf])

                    c_boxes.append(torch.tensor(pixels))
                    c_labels.append(f"Class: {cls},"
                                    f"Meters: {round(abs_distance, 2)},"
                                    f"({round(float(obj_dist1[0]), 2)},"
                                    f"{round(float(obj_dist1[1]), 2)},"
                                    f"{round(float(obj_dist1[2]), 2)})")

        """print("DISTANCE_ARRAY: ", distance_output)"""
        self.distance_publisher.publish(
           Float32MultiArray(data=distance_output))

        """transposed_image = np.transpose(cv_image, (2, 0, 1))
        image_np_with_detections = torch.tensor(transposed_image,
                                                dtype=torch.uint8)"""

        if 9 in output[0].boxes.cls:
            self.process_traffic_lights(output[0], cv_image, image.header)

        """c_boxes = torch.stack(c_boxes)
        box = draw_bounding_boxes(image_np_with_detections,
                                  c_boxes,
                                  c_labels,
                                  colors='blue',
                                  width=3,
                                  font_size=12)
        np_box_img = np.transpose(box.detach().numpy(),
                                  (1, 2, 0))
        box_img = cv2.cvtColor(np_box_img, cv2.COLOR_BGR2RGB)"""
        locals().clear()
        # return box_img

        # return output[0].plot()

    def process_traffic_lights(self, prediction, cv_image, image_header):
        indices = (prediction.boxes.cls == 9).nonzero().squeeze().cpu().numpy()
        indices = np.asarray([indices]) if indices.size == 1 else indices

        max_y = 360  # middle of image
        min_prob = 0.30

        for index in indices:
            box = prediction.boxes.cpu().data.numpy()[index]

            if box[4] < min_prob:
                continue

            if (box[2] - box[0]) * 1.5 > box[3] - box[1]:
                continue  # ignore horizontal boxes

            if box[1] > max_y:
                continue

            box = box[0:4].astype(int)
            segmented = cv_image[box[1]:box[3], box[0]:box[2]]

            traffic_light_image = self.bridge.cv2_to_imgmsg(segmented,
                                                            encoding="rgb8")
            traffic_light_image.header = image_header
            self.traffic_light_publisher.publish(traffic_light_image)

        locals().clear()

    def create_mask(self, input_image, model_output):
        output_predictions = torch.argmax(model_output, dim=0)
        for i in range(21):
            output_predictions[i] = output_predictions[i] == i

        output_predictions = output_predictions.to(dtype=torch.bool)

        transposed_image = np.transpose(input_image, (2, 0, 1))
        tensor_image = torch.tensor(transposed_image)
        tensor_image = tensor_image.to(dtype=torch.uint8)
        segmented_image = draw_segmentation_masks(tensor_image,
                                                  output_predictions,
                                                  alpha=0.6)
        cv_segmented = segmented_image.detach().cpu().numpy()
        cv_segmented = np.transpose(cv_segmented, (1, 2, 0))
        return cv_segmented

    def apply_bounding_boxes(self, input_image, model_output):
        transposed_image = np.transpose(input_image, (2, 0, 1))
        image_np_with_detections = torch.tensor(transposed_image,
                                                dtype=torch.uint8)
        boxes = model_output['boxes']
        # scores = model_output['scores']
        labels = [self.weights.meta["categories"][i]
                  for i in model_output['labels']]

        box = draw_bounding_boxes(image_np_with_detections,
                                  boxes,
                                  labels,
                                  colors='blue',
                                  width=3,
                                  font_size=24)

        np_box_img = np.transpose(box.detach().numpy(),
                                  (1, 2, 0))
        box_img = cv2.cvtColor(np_box_img, cv2.COLOR_BGR2RGB)
        return box_img

    def run(self):
        self.spin()
        pass


if __name__ == "__main__":
    roscomp.init("VisionNode")
    node = VisionNode("VisionNode")
    node.run()

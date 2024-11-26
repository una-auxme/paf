#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
import torch
import numpy as np
import cv2
import math
from cv_bridge import CvBridge


def region_of_interest(img, vertices):
    mask = np.zeros_like(img)
    match_mask_color = 255
    cv2.fillPoly(mask, vertices, match_mask_color)
    masked_image = cv2.bitwise_and(img, mask)
    return masked_image


def draw_lines(img, lines, color=[255, 0, 0], thickness=3):
    line_img = np.zeros((img.shape[0], img.shape[1], 3), dtype=np.uint8)
    img = np.copy(img)

    if lines is None:
        return img
    for line in lines:
        for x1, y1, x2, y2 in line:
            cv2.line(line_img, (x1, y1), (x2, y2), color, thickness)
    img = cv2.addWeighted(img, 0.8, line_img, 1.0, 0.0)
    return img, line_img


class LaneDetection(CompatibleNode):

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.bridge = CvBridge()
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic="/carla/hero/Center/image",
            qos_profile=1,
        )

        self.publisher_mask = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic="/paf/hero/Lane_Detection_Mask",
            qos_profile=1,
        )
        self.publisher_image = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic="/paf/hero/Lane_Detection_Image",
            qos_profile=1,
        )

    def run(self):
        self.spin()

    def handle_camera_image(self, msg_image):
        if self.device == "cuda":
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        image = self.bridge.imgmsg_to_cv2(img_msg=msg_image, desired_encoding="rgb8")
        prediction = self.detection(image)

        img_msg_mask = self.bridge.cv2_to_imgmsg(prediction[1], encoding="rgb8")
        img_msg_mask.header = msg_image.header
        self.publisher_mask.publish(img_msg_mask)

        img_msg_image = self.bridge.cv2_to_imgmsg(prediction[0], encoding="rgb8")
        img_msg_image.header = msg_image.header
        self.publisher_image.publish(img_msg_image)

    def detection(self, image):
        height = image.shape[0]
        width = image.shape[1]
        region_of_interest_vertices = [
            (300, height - 150),
            (width / 2 - 200, height / 2),
            (width / 2 + 200, height / 2),
            (width - 300, height - 150),
        ]

        gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
        cannyed_image = cv2.Canny(gray_image, 100, 200)

        cropped_image = region_of_interest(
            cannyed_image,
            np.array([region_of_interest_vertices], np.int32),
        )

        lines = cv2.HoughLinesP(
            cropped_image,
            rho=6,
            theta=np.pi / 60,
            threshold=160,
            lines=np.array([]),
            minLineLength=40,
            maxLineGap=25,
        )

        left_line_x = []
        left_line_y = []
        right_line_x = []
        right_line_y = []

        for line in lines:
            for x1, y1, x2, y2 in line:
                slope = (y2 - y1) / (x2 - x1)
                if math.fabs(slope) < 0.5:
                    continue
                if slope <= 0:
                    left_line_x.extend([x1, x2])
                    left_line_y.extend([y1, y2])
                else:
                    right_line_x.extend([x1, x2])
                    right_line_y.extend([y1, y2])

        min_y = int(image.shape[0] * (3 / 5))
        max_y = int(image.shape[0])
        poly_left = np.poly1d(np.polyfit(left_line_y, left_line_x, deg=1))

        left_x_start = int(poly_left(max_y))
        left_x_end = int(poly_left(min_y))

        poly_right = np.poly1d(np.polyfit(right_line_y, right_line_x, deg=1))

        right_x_start = int(poly_right(max_y))
        right_x_end = int(poly_right(min_y))
        image, line_image = draw_lines(
            image,
            [
                [
                    [left_x_start, max_y, left_x_end, min_y],
                    [right_x_start, max_y, right_x_end, min_y],
                ]
            ],
            thickness=5,
        )
        return image, line_image


if __name__ == "__main__":
    roscomp.init("LaneDetection")
    node = LaneDetection("LaneDetection")
    node.run()

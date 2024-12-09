#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
import torch
import rospy
import numpy as np
import cv2
import math
from cv_bridge import CvBridge
from visualization_msgs.msg import Marker, MarkerArray


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
        self.dist_arrays = None
        self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic="/carla/hero/Center/image",
            qos_profile=1,
        )
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_dist_array,
            topic="/paf/hero/Center/dist_array",
            qos_profile=1,
        )

        self.marker_publisher = self.new_publisher(
            msg_type=MarkerArray,
            topic="/paf/hero/visualization_marker_array_lane",
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
        self.publisher_dist = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic="/paf/hero/Lane_Detection_Image_dist",
            qos_profile=1,
        )

    def run(self):
        self.spin()

    def get_marker(self, point, id):
        c_color = [157, 234, 50]
        marker = Marker()
        marker.header.frame_id = "hero"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "min_values"
        marker.id = id
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.01
        marker.color.a = 1.0
        marker.color.r = c_color[0]
        marker.color.g = c_color[1]
        marker.color.b = c_color[2]
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = point[0]
        marker.pose.position.y = point[1]
        marker.pose.position.z = 1.7 + point[2]
        marker.lifetime = rospy.Duration(0.1)
        return marker

    def handle_camera_image(self, msg_image):
        if self.device == "cuda":
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
        image = self.bridge.imgmsg_to_cv2(img_msg=msg_image, desired_encoding="rgb8")
        prediction = self.detection(image)
        mask = prediction[1]
        markers = MarkerArray()
        if self.dist_arrays is not None:
            disk_mask = np.any(self.dist_arrays, axis=2, keepdims=True)
            mask = mask & disk_mask
            dist_mask = np.logical_and(
                np.any(mask, axis=2, keepdims=True), self.dist_arrays
            )
            dist = dist_mask * self.dist_arrays
            points = dist[np.repeat(disk_mask, 3, axis=2)].reshape(-1, 3)
            total_points_counter = 0
            wrong_points_counter = 0
            for i, point in enumerate(points):
                if point[0] > 2:
                    if point[2] < -1.5:
                        total_points_counter += 1
                        markers.markers.append(self.get_marker(point, i))
                    else:
                        total_points_counter += 1
                        wrong_points_counter += 1

        if wrong_points_counter / total_points_counter < 0.2:
            self.marker_publisher.publish(markers)
        else:
            rospy.loginfo("no lane detection available")
        img_msg_mask = self.bridge.cv2_to_imgmsg(prediction[1], encoding="rgb8")
        img_msg_mask.header = msg_image.header
        self.publisher_mask.publish(img_msg_mask)
        img_msg_image = self.bridge.cv2_to_imgmsg(prediction[0], encoding="rgb8")
        img_msg_image.header = msg_image.header
        self.publisher_image.publish(img_msg_image)

    def min_x(self, dist_array):
        """
        Calculate min x (distance forward)
        Args:
            dist_array (np array): numpy array containing all
            lidar point in one bounding box
        Returns:
            np.array: 1x3 numpy array of min x lidar point
        """
        min_x_sorted_indices = np.argsort(dist_array[:, :, 0], axis=None)
        x, y = np.unravel_index(min_x_sorted_indices[0], dist_array.shape[:2])
        return dist_array[x][y].copy()

    def handle_dist_array(self, dist_array):
        """
        This function overwrites the current depth image from
        the lidar distance node with the latest depth image.

        Args:
            dist_array (image msg): Depth image frim Lidar Distance Node
        """
        # callback function for lidar depth imagemask
        # since frequency is lower than image frequency
        # the latest lidar image is saved
        dist_array = self.bridge.imgmsg_to_cv2(
            img_msg=dist_array, desired_encoding="passthrough"
        )
        self.dist_arrays = dist_array

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
            thickness=10,
        )
        return image, line_image


if __name__ == "__main__":
    roscomp.init("LaneDetection")
    node = LaneDetection("LaneDetection")
    node.run()

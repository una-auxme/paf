#!/usr/bin/env python3

from datetime import datetime
import threading
from time import sleep
from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from perception.msg import TrafficLightState
from std_msgs.msg import Int16
from cv_bridge import CvBridge
from traffic_light_detection.src.traffic_light_detection.traffic_light_inference import (  # noqa: E501
    TrafficLightInference,
)
import cv2
import numpy as np

import rospy
from visualization_msgs.msg import Marker


class TrafficLightNode(CompatibleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        # general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.classifier = TrafficLightInference(self.get_param("model", ""))
        self.last_info_time: datetime = None
        self.last_state = None
        self.visual_debug = True
        threading.Thread(target=self.auto_invalidate_state).start()

        # publish / subscribe setup
        self.setup_camera_subscriptions()
        self.setup_traffic_light_publishers()

    def setup_camera_subscriptions(self):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic=f"/paf/{self.role_name}/{self.side}/segmented_traffic_light",
            qos_profile=1,
        )

    def setup_traffic_light_publishers(self):
        self.traffic_light_publisher = self.new_publisher(
            msg_type=TrafficLightState,
            topic=f"/paf/{self.role_name}/{self.side}/traffic_light_state",
            qos_profile=1,
        )

        self.traffic_light_distance_publisher = self.new_publisher(
            msg_type=Int16,
            topic=f"/paf/{self.role_name}/{self.side}" + "/traffic_light_y_distance",
            qos_profile=1,
        )

        self.marker_pub = rospy.Publisher(
            "/paf/hero/TrafficLight/state/debug_marker", Marker, queue_size=10
        )

        self.traffic_light_masked = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/masked_traffic_light",
            qos_profile=1,
        )

    def auto_invalidate_state(self):
        while True:
            sleep(1)

            if self.last_info_time is None:
                continue

            if (datetime.now() - self.last_info_time).total_seconds() >= 2:
                msg = TrafficLightState()
                msg.state = 0
                self.traffic_light_publisher.publish(msg)
                if self.visual_debug:
                    traffic_light_visualization(self, msg.state)
                self.traffic_light_distance_publisher.publish(Int16(0))
                self.last_info_time = None

    def handle_camera_image(self, image):
        distance = int(image.header.frame_id)

        cv2_image = self.bridge.imgmsg_to_cv2(image)
        rgb_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
        result, data = self.classifier(cv2_image)

        if (
            data[0][0] > 1e-15
            and data[0][3] > 1e-15
            or data[0][0] > 1e-10
            or data[0][3] > 1e-10
        ):
            return  # too uncertain, may not be a traffic light

        if not is_front(self, rgb_image):
            return  # not a front facing traffic light

        # mask_image = cv2.cvtColor(mask, cv2.COLOR_BGR2RGB)
        traffic_light_image = self.bridge.cv2_to_imgmsg(rgb_image, encoding="rgb8")
        self.traffic_light_masked.publish(traffic_light_image)

        state = result if result in [1, 2, 4] else 0
        if self.last_state == state:
            # 1: Green, 2: Red, 4: Yellow, 0: Unknown
            msg = TrafficLightState()
            msg.state = state
            if self.visual_debug:
                traffic_light_visualization(self, state)
            self.traffic_light_publisher.publish(msg)

            if distance is not None:
                # https://photo.stackexchange.com/questions/12434/how-do-i-calculate-the-distance-of-an-object-in-a-photo
                self.traffic_light_distance_publisher.publish(Int16(distance))
        else:
            self.last_state = state

        # Automatically invalidates state (state=0) in auto_invalidate_state()
        if state != 0:
            self.last_info_time = datetime.now()

    def run(self):
        self.spin()


def traffic_light_visualization(self, state):
    print("i want to publish")
    text_marker = Marker()
    text_marker.header.frame_id = "hero"
    text_marker.header.stamp = rospy.Time.now()
    text_marker.ns = "traffic_light"
    text_marker.id = 1
    text_marker.type = Marker.TEXT_VIEW_FACING
    text_marker.action = Marker.ADD

    text_marker.scale.z = 0.5  # Textsize
    if state == 0:
        text_marker.text = "TLS: Unknown"
        text_marker.color.r = 0.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
    if state == 1:
        text_marker.text = "TLS: Green"
        text_marker.color.r = 0.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0
    if state == 2:
        text_marker.text = "TLS: Red"
        text_marker.color.r = 1.0
        text_marker.color.g = 0.0
        text_marker.color.b = 0.0
    if state == 4:
        text_marker.text = "TLS: Yellow"
        text_marker.color.r = 1.0
        text_marker.color.g = 1.0
        text_marker.color.b = 0.0

    text_marker.color.a = 1.0

    # Position des Textmarkers (oberhalb des Pfeils)
    text_marker.pose.position.x = 0.0
    text_marker.pose.position.y = 0.0
    text_marker.pose.position.z = 2.0  # show over vehicle

    # Publish the marker
    self.marker_pub.publish(text_marker)


def get_light_mask(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define lower and upper bounds for the hue, saturation, and value
    # - Red and Yellow combined since they are so close in the color spectrum
    lower_red_yellow = np.array([0, 75, 100])
    upper_red_yellow = np.array([40, 255, 255])
    lower_green = np.array([40, 200, 200])
    upper_green = np.array([80, 255, 255])

    # Mask where the pixels within the bounds are white, otherwise black
    m1 = cv2.inRange(hsv, lower_red_yellow, upper_red_yellow)
    m2 = cv2.inRange(hsv, lower_green, upper_green)
    mask = cv2.bitwise_or(m1, m2)

    return mask


def is_front(self, image):
    mask = get_light_mask(image)

    # Find contours in the thresholded image, use only the largest one
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = sorted(contours, key=cv2.contourArea, reverse=True)[:1]
    contour = contours[0] if contours else None

    if contour is None:
        return False

    _, _, width, height = cv2.boundingRect(contour)
    aspect_ratio = width / height

    # If aspect ratio is within range of a square (therefore a circle)
    if 0.75 <= aspect_ratio <= 1.3:
        return True
    else:
        return False


if __name__ == "__main__":
    roscomp.init("TrafficLightNode")
    node = TrafficLightNode("TrafficLightNode")
    node.run()

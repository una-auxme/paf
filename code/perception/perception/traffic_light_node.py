from typing import List
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from sensor_msgs.msg import Image as ImageMsg
from perception_interfaces.msg import TrafficLightState, TrafficLightImages
from cv_bridge import CvBridge
import cv2
import numpy as np

from visualization_msgs.msg import Marker

from paf_common.parameters import update_attributes
from paf_common.exceptions import emsg_with_trace
from rclpy.parameter import Parameter

from traffic_light_detection.src.traffic_light_inference import (
    TrafficLightInference,
)


class TrafficLightNode(Node):
    def __init__(self):
        super().__init__(type(self).__name__)
        self.get_logger().info(f"{type(self).__name__} node initializing...")

        # Parameters

        self.control_loop_rate = (
            self.declare_parameter(
                "control_loop_rate",
                0.05,
            )
            .get_parameter_value()
            .double_value
        )
        self.role_name = (
            self.declare_parameter("role_name", "hero")
            .get_parameter_value()
            .string_value
        )
        self.side = (
            self.declare_parameter("side", "Center").get_parameter_value().string_value
        )
        self.model = (
            self.declare_parameter("model", "").get_parameter_value().string_value
        )
        self.tfs_debug = (
            self.declare_parameter("tfs_debug", False).get_parameter_value().bool_value
        )

        # general setup
        self.bridge = CvBridge()

        self.classifier = TrafficLightInference(self.model)
        self.last_info_time = self.get_clock().now()
        self.traffic_light_msg = TrafficLightState()
        self.traffic_light_msg.state = 0

        # publish / subscribe setup
        self.setup_camera_subscriptions()
        self.setup_traffic_light_publishers()

        self.create_timer(self.control_loop_rate, self.loop_handler)
        self.add_on_set_parameters_callback(self._set_parameters_callback)
        self.get_logger().info(f"{type(self).__name__} node initialized.")

    def _set_parameters_callback(self, params: List[Parameter]):
        """Callback for parameter updates."""
        return update_attributes(self, params)

    def setup_camera_subscriptions(self):
        """receives images and runs handel_camera_image"""
        self.create_subscription(
            msg_type=TrafficLightImages,
            callback=self.handle_camera_image,
            topic=f"/paf/{self.role_name}/{self.side}/segmented_traffic_light",
            qos_profile=1,
        )

    def setup_traffic_light_publishers(self):
        # publishes current state of traffic light
        self.traffic_light_publisher = self.create_publisher(
            msg_type=TrafficLightState,
            topic=f"/paf/{self.role_name}/{self.side}/traffic_light_state",
            qos_profile=1,
        )
        # publishes a debug visualization of the current state
        self.marker_pub = self.create_publisher(
            Marker, "/paf/hero/TrafficLight/state/debug_marker", 10
        )

    def handle_camera_image(self, msg: TrafficLightImages):
        
        """
        # calculates the current state of the traffic light
        cv2_image = self.bridge.imgmsg_to_cv2(image)
        rgb_image = cv2.cvtColor(cv2_image, cv2.COLOR_BGR2RGB)
        # apply a NN on the image to determine state
        result, data = self.classifier(cv2_image)

        if (
            data[0][0] > 1e-15
            and data[0][3] > 1e-15
            or data[0][0] > 1e-10
            or data[0][3] > 1e-10
        ):
            return  # too uncertain, may not be a traffic light
        # checks if the traffic light has correct orientation
        if not is_front(rgb_image):
            return  # not a front facing traffic light

        # 1: Green, 2: Red, 4: Yellow other values (back or side of traffic light) are
        # interpreted as unknown
        state = result if result in [1, 2, 4] else 0
        self.traffic_light_msg.state = state
        if state != 0:
            self.last_info_time = self.get_clock().now() """

        self.get_logger().info(
        f"Received {len(msg.images)} traffic light images"
        )        

    def loop(self):
        # check if the last state was received more than 2 seconds ago
        if (
            self.last_info_time + Duration(seconds=2.0) < self.get_clock().now()
            and self.traffic_light_msg.state != 0
        ):
            self.traffic_light_msg.state = 0
        self.traffic_light_publisher.publish(self.traffic_light_msg)
        if self.tfs_debug:
            self.traffic_light_visualization(self.traffic_light_msg.state)

    def loop_handler(self):
        try:
            self.loop()
        except Exception as e:
            self.get_logger().fatal(emsg_with_trace(e), throttle_duration_sec=2)

    def traffic_light_visualization(self, state):
        # pulishes a debug visualization of the current state
        text_marker = Marker()
        text_marker.header.frame_id = "hero"
        text_marker.header.stamp = self.get_clock().now().to_msg()
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

        # Position of the marker
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


def is_front(image):
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


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrafficLightNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

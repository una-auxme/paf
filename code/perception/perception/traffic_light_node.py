from typing import List
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from perception_interfaces.msg import TrafficLightState, TrafficLightImages
from cv_bridge import CvBridge
import cv2
import numpy as np

from visualization_msgs.msg import Marker

from paf_common.parameters import update_attributes
from paf_common.exceptions import emsg_with_trace
from rclpy.parameter import Parameter
from collections import deque

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
        self.state_buffer = deque(maxlen=10)
        self.last_state = 0

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
        # Classifies all traffic light images that appear per frame
        # These are passed via the TrafficLightImages message type
        results = []
        state_found = False

        # Only one state != 0 may occur, otherwise the last state is taken
        for image_msg in msg.images:
            cv_image = self.bridge.imgmsg_to_cv2(image_msg, "rgb8")
            result = self.classifier(cv_image)
            if state_found and result != 0:
                results.clear()
                results.append(self.last_state)
                break
            elif self.filter_turn_lights(cv_image) and not state_found:
                results.append(result)

            if result != 0:
                state_found = True

        if not results:
            return

        if 2 in results:
            interim_state = 2
        elif 1 in results:
            interim_state = 1
        elif 4 in results:
            interim_state = 4
        else:
            interim_state = 0

        # Cache values and only change state after a certain number has been reached
        self.state_buffer.append(interim_state)
        if self.state_buffer.count(2) >= 4:
            state = 2
            self.last_state = 2
        elif self.state_buffer.count(1) >= 5:
            state = 1
            self.last_state = 1
        elif self.state_buffer.count(4) >= 4:
            state = 4
            self.last_state = 4

        else:
            state = self.last_state

        self.traffic_light_msg.state = state
        if state != 0:
            self.last_info_time = self.get_clock().now()

        self.get_logger().info(
            f"TL : {tuple(results)} -> {tuple(self.state_buffer)} -> {state}"
        )

    def loop(self):
        # check if the last state was received more than 2 seconds ago
        if (
            self.last_info_time + Duration(seconds=2.0) < self.get_clock().now()
            and self.traffic_light_msg.state != 0
        ):
            self.traffic_light_msg.state = 0
            self.last_state = 0
            self.state_buffer.clear()
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

    def filter_turn_lights(self, image):
        # Traffic lights that are detected when turning are ignored
        # Orginalbild
        cv2.imshow("input", image)

        # Bild in Graustufen darstellen
        gray = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        circles = cv2.HoughCircles(
            gray,
            cv2.HOUGH_GRADIENT,
            dp=1.0,
            minDist=35,
            param1=150,
            param2=7,
            minRadius=2,
            maxRadius=7,
        )

        if circles is not None:
            for x, y, r in circles[0]:
                x = int(round(x))
                y = int(round(y))
                r = int(round(r))

                cv2.circle(image, (x, y), r, (0, 255, 0), 2)
                cv2.circle(image, (x, y), 2, (255, 0, 0), 3)

        cv2.imshow("circles", image)
        cv2.waitKey(1)

        # 
        if circles is not None and len(circles[0]) >= 2:
            return False
        else:
            return True


def main(args=None):
    rclpy.init(args=args)

    try:
        node = TrafficLightNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()

#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from perception.msg import TrafficLightState
from std_msgs.msg import Header
from cv_bridge import CvBridge
from traffic_light_detection.src.traffic_light_detection.traffic_light_inference import TrafficLightInference


class TrafficLightNode(CompatibleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        # general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.classifier = TrafficLightInference(self.get_param("model", ""))

        # publish / subscribe setup
        self.setup_camera_subscriptions()
        self.setup_traffic_light_publishers()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

    def setup_camera_subscriptions(self):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic=f"/paf/{self.role_name}/{self.side}/segmented_traffic_light",
            qos_profile=1
        )

    def setup_traffic_light_publishers(self):
        self.traffic_light_publisher = self.new_publisher(
            msg_type=TrafficLightState,
            topic=f"/paf/{self.role_name}/{self.side}/traffic_light_state",
            qos_profile=1
        )

    def handle_camera_image(self, image):
        result = self.classifier(self.bridge.imgmsg_to_cv2(image))

        msg = TrafficLightState()
        msg.isGreen = result == 1
        msg.isYellow = result == 4
        msg.isRed = result == 2

        print(f"Traffic light state: {result}")

        self.traffic_light_publisher.publish(msg)

    def run(self):
        self.spin()


if __name__ == "__main__":
    roscomp.init("TrafficLightNode")
    node = TrafficLightNode("TrafficLightNode")
    node.run()

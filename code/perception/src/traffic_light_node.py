#!/usr/bin/env python3

from datetime import datetime
import threading
from time import sleep
from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
from perception.msg import TrafficLightState
from cv_bridge import CvBridge
from traffic_light_detection.src.traffic_light_detection.traffic_light_inference \
    import TrafficLightInference  # noqa: E501


class TrafficLightNode(CompatibleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        # general setup
        self.bridge = CvBridge()
        self.role_name = self.get_param("role_name", "hero")
        self.side = self.get_param("side", "Center")
        self.classifier = TrafficLightInference(self.get_param("model", ""))
        self.last_info: datetime = None
        threading.Thread(target=self.auto_invalidate_state).start()

        # publish / subscribe setup
        self.setup_camera_subscriptions()
        self.setup_traffic_light_publishers()

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

    def auto_invalidate_state(self):
        while True:
            sleep(1)

            if self.last_info is None:
                continue

            if (datetime.now() - self.last_info).total_seconds() >= 3:
                msg = TrafficLightState()
                msg.state = 0
                self.traffic_light_publisher.publish(msg)
                self.last_info = None

    def handle_camera_image(self, image):
        result = self.classifier(self.bridge.imgmsg_to_cv2(image))

        # 1: Green, 2: Red, 4: Yellow, 0: Unknown
        msg = TrafficLightState()
        msg.state = result if result in [1, 2, 4] else 0
        self.traffic_light_publisher.publish(msg)

        # invalidates state (state=0) after 3s in auto_invalidate_state()
        self.last_info = datetime.now()

    def run(self):
        self.spin()


if __name__ == "__main__":
    roscomp.init("TrafficLightNode")
    node = TrafficLightNode("TrafficLightNode")
    node.run()

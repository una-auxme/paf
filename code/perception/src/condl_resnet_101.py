#!/usr/bin/env python3

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
import rospy


class LaneDetection(CompatibleNode):

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.handle_camera_image,
            topic="/carla/hero/Center/image",
            qos_profile=1,
        )

        self.traffic_light_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/hero/Lane_Detection_Mask",
            qos_profile=1,
        )

    def run(self):
        self.spin()

    def handle_camera_image(self, image):
        rospy.loginfo("yhea")


if __name__ == "__main__":
    roscomp.init("LaneDetection")
    node = LaneDetection("LaneDetection")
    node.run()

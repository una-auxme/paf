#!/usr/bin/env python

from ros_compatibility.node import CompatibleNode
import ros_compatibility as roscomp
from cv_bridge import CvBridge
from rospy.numpy_msg import numpy_msg
from sensor_msgs.msg import Image as ImageMsg
import numpy as np


class lane_position(CompatibleNode):

    def __init__(self, name, **kwargs):
        super().__init(name, **kwargs)
        self.bridge = CvBridge()

        self.setup_lanemask_subscription()

    def setup_lanemask_subscription(self):
        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.lanemask_handler,
            topic="/carla/hero/Center/lane_mask",
            qos_profile=1,
        )

    def lanemask_handler(self, ImageMsg):

        self.lanemask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg)

    def transform_camera2world(self,mask):
        

if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = lane_position("lane_position")
    node.run()

#!/usr/bin/env python
# ROS imports
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image as ImageMsg
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge


class lane_position(CompatibleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.bridge = CvBridge()
        # get camera parameters
        self.camera_x = self.get_param("camera_x")
        self.setup_lanemask_subscriptions()

    def run(self):
        self.spin()
        pass

    def setup_lanemask_subscriptions(self):
        """
        sets up a subscriber to the lanemask
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.lanemask_handler,
            topic="/paf/hero/Center/lane_mask",
            qos_profile=1,
        )

    def lanemask_handler(self, ImageMsg):
        self.lanemask = self.bridge.imgmsg_to_cv2(
            img_msg=ImageMsg, desired_encoding="mono8"
        )


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = lane_position("Lanedetection_node")
    node.run()

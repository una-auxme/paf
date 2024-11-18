#!/usr/bin/env python

# import pickle
from ros_compatibility.node import CompatibleNode

from rospy.numpy_msg import numpy_msg

# import rospy

# import numpy as np
import ros_compatibility as roscomp

# import cv2
from sensor_msgs.msg import Image as ImageMsg
from std_msgs.msg import Header

# import torch

from cv_bridge import CvBridge


class CLRerNet(CompatibleNode):
    """CLRerNet:
    Model for Lanedetection

    subscribes to camera image and publishes lane masks for further planning
    """

    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)

        self.bridge = CvBridge()
        self.image_msg_header = Header()
        self.image_msg_header.frame_id = "segmented_image_frame"

        self.role_name = self.get_param("role_name", "hero")
        self.setup_camera_subscriptions("Center")
        self.setup_lane_publisher()

    def run(self):
        self.spin()
        pass

    def setup_camera_subscriptions(self, side):
        """
        sets up a subscriber to the selected camera angle

        Args:
            side (String): Camera angle specified in launch file
        """

        self.new_subscription(
            msg_type=numpy_msg(ImageMsg),
            callback=self.image_handler,
            topic=f"/carla/{self.role_name}/{side}/image",
            qos_profile=1,
        )

    def image_handler(self, ImageMsg):
        self.lane_publisher.publish(ImageMsg)

    def setup_lane_publisher(self):
        self.lane_publisher = self.new_publisher(
            msg_type=numpy_msg(ImageMsg),
            topic=f"/paf/{self.role_name}/Center/lane_img",
            qos_profile=1,
        )

    """
    def setup_mask_publisher(self):

        sets up a publisher to the selected camera angle
        if multiple cameras are enabled, there are multiple publishers used


        if self.center:
            self.publisher_center = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Center/lane_mask",
                qos_profile=1,
            )
        if self.back:
            self.publisher_back = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Back/lane_mask",
                qos_profile=1,
            )
        if self.left:
            self.publisher_left = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Left/lane_mask",
                qos_profile=1,
            )
        if self.right:
            self.publisher_right = self.new_publisher(
                msg_type=numpy_msg(ImageMsg),
                topic=f"/paf/{self.role_name}/Right/lane_mask",
                qos_profile=1,
            )
    """


if __name__ == "__main__":
    roscomp.init("CLRerNet")
    node = CLRerNet("CLRerNet")
    node.run()

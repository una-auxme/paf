#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2

# from mpl_toolkits.mplot3d import Axes3D
# from itertools import combinations
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge

from std_msgs.msg import Float32

# from matplotlib.colors import LinearSegmentedColormap


class RadarNode:
    """See doc/perception/lidar_distance_utility.md on
    how to configute this node
    """

    def callback(self, data):
        """Callback function, filters a PontCloud2 message
            by restrictions defined in the launchfile.

            Publishes a Depth image for the specified camera angle.
            Each angle has do be delt with differently since the signs of the
            coordinate system change with the view angle.

        :param data: a PointCloud2
        """

        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        msg = np.min(coordinates["Velocity"])
        self.dist_array_radar_publisher.publish(msg)

    def listener(self):
        """
        Initializes the node and it's publishers
        """
        # run simultaneously.
        rospy.init_node("lidar_distance")
        self.bridge = CvBridge()

        # publisher for radar dist_array
        self.dist_array_radar_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Radar/velocity"),
            Float32,
            queue_size=10,
        )

        rospy.Subscriber(
            rospy.get_param("~source_topic", "/carla/hero/RADAR"),
            PointCloud2,
            self.callback,
        )

        rospy.spin()


if __name__ == "__main__":
    lidar_distance = RadarNode()
    lidar_distance.listener()

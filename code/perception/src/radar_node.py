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
        # coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        # Center

        # reconstruct_bit_mask_center = lidar_filter_utility.bounding_box(
        #     coordinates,
        #     max_x=np.inf,
        #     min_x=0.0,
        #     min_z=-1.6,
        # )
        # reconstruct_coordinates_center = coordinates[reconstruct_bit_mask_center]
        # reconstruct_coordinates_xyz_center = np.array(
        #     lidar_filter_utility.remove_field_name(
        #         reconstruct_coordinates_center, "intensity"
        #     ).tolist()
        # )
        # dist_array_center = self.reconstruct_img_from_lidar(
        #     reconstruct_coordinates_xyz_center, focus="Center"
        # )
        # dist_array_center_msg = self.bridge.cv2_to_imgmsg(
        #     dist_array_center, encoding="passthrough"
        # )
        # dist_array_center_msg.header = data.header
        # self.dist_array_center_publisher.publish(dist_array_center_msg)

        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        # x_values = coordinates['x']
        # rospy.loginfo("Erste 5 x Werte: {x_values[:5]}")

        # depth_values = coordinates['depth']
        # rospy.loginfo("Erste 5 Depth Werte: {depth_values[:5]}")

        dtype_info = "\n".join([f"Feld '{name}': {coordinates.dtype[name]}" for name in coordinates.dtype.names])

        # rospy.loginfo("DatentypenNeu: " + dtype_info) # funktioniert

        # rospy.loginfo("DatentypenAlt: " + coordinates.dtype)

        

        msg = np.min[coordinates['Range']]

        dtype_msginfo = ["{type(msg).__name__}"]
        rospy.loginfo("DatentypenMsg: " + dtype_msginfo)

        self.dist_array_radar_publisher.publish(msg)      

    def listener(self):
        """
        Initializes the node and it's publishers
        """
        # run simultaneously.
        rospy.init_node("lidar_distance")
        self.bridge = CvBridge()

        self.pub_pointcloud = rospy.Publisher(
            rospy.get_param(
                "~point_cloud_topic",
                "/carla/hero/" + rospy.get_namespace() + "_filtered",
            ),
            PointCloud2,
            queue_size=10,
        )

        # publisher for dist_array

        # self.dist_array_center_publisher = rospy.Publisher(
        #     rospy.get_param("~image_distance_topic", "/paf/hero/Center/dist_array"),
        #     ImageMsg,
        #     queue_size=10,
        # )

        # publisher for radar dist_array
        self.dist_array_radar_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Radar/array"),
            # PointCloud2,
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

#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Float32


class RadarNode:
    """See doc/perception/radar_node.md on
    how to configure this node
    """

    def callback(self, data):
        """Process radar Point2Cloud data and publish minimum velocity.

        Extracts velocity information from radar data
        and publishes the minimum velocity as a
        Float32 message

        Args:
            data: Point2Cloud message containing radar data with velocity field
        """

        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)
        msg = np.min(coordinates["Velocity"])
        self.dist_array_radar_publisher.publish(msg)

    def listener(self):
        """
        Initializes the node and it's publishers
        """
        # run simultaneously.
        rospy.init_node("radar_node")

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
    radar_node = RadarNode()
    radar_node.listener()

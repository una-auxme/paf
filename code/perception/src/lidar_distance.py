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

# from matplotlib.colors import LinearSegmentedColormap


class LidarDistance:
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

        # Center
        reconstruct_bit_mask_center = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=np.inf,
            min_x=0.0,
            min_z=-1.6,
        )
        reconstruct_coordinates_center = coordinates[reconstruct_bit_mask_center]
        reconstruct_coordinates_xyz_center = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_center, "intensity"
            ).tolist()
        )
        dist_array_center = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_center, focus="Center"
        )
        dist_array_center_msg = self.bridge.cv2_to_imgmsg(
            dist_array_center, encoding="passthrough"
        )
        dist_array_center_msg.header = data.header
        self.dist_array_center_publisher.publish(dist_array_center_msg)

        # Back
        reconstruct_bit_mask_back = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=0.0,
            min_x=-np.inf,
            min_z=-1.6,
        )
        reconstruct_coordinates_back = coordinates[reconstruct_bit_mask_back]
        reconstruct_coordinates_xyz_back = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_back, "intensity"
            ).tolist()
        )
        dist_array_back = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_back, focus="Back"
        )
        dist_array_back_msg = self.bridge.cv2_to_imgmsg(
            dist_array_back, encoding="passthrough"
        )
        dist_array_back_msg.header = data.header
        self.dist_array_back_publisher.publish(dist_array_back_msg)

        # Left
        reconstruct_bit_mask_left = lidar_filter_utility.bounding_box(
            coordinates,
            max_y=np.inf,
            min_y=0.0,
            min_z=-1.6,
        )
        reconstruct_coordinates_left = coordinates[reconstruct_bit_mask_left]
        reconstruct_coordinates_xyz_left = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_left, "intensity"
            ).tolist()
        )
        dist_array_left = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_left, focus="Left"
        )
        dist_array_left_msg = self.bridge.cv2_to_imgmsg(
            dist_array_left, encoding="passthrough"
        )
        dist_array_left_msg.header = data.header
        self.dist_array_left_publisher.publish(dist_array_left_msg)

        # Right
        reconstruct_bit_mask_right = lidar_filter_utility.bounding_box(
            coordinates, max_y=-0.0, min_y=-np.inf, min_z=-1.6
        )
        reconstruct_coordinates_right = coordinates[reconstruct_bit_mask_right]
        reconstruct_coordinates_xyz_right = np.array(
            lidar_filter_utility.remove_field_name(
                reconstruct_coordinates_right, "intensity"
            ).tolist()
        )
        dist_array_right = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz_right, focus="Right"
        )
        dist_array_right_msg = self.bridge.cv2_to_imgmsg(
            dist_array_right, encoding="passthrough"
        )
        dist_array_right_msg.header = data.header
        self.dist_array_right_publisher.publish(dist_array_right_msg)

    def listener(self):
        """
        Initializes the node and it's publishers
        """
        # run simultaneously.
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
        self.dist_array_center_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Center/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        # publisher for dist_array
        self.dist_array_back_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Back/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        # publisher for dist_array
        self.dist_array_left_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Left/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        # publisher for dist_array
        self.dist_array_right_publisher = rospy.Publisher(
            rospy.get_param("~image_distance_topic", "/paf/hero/Right/dist_array"),
            ImageMsg,
            queue_size=10,
        )

        rospy.Subscriber(
            rospy.get_param("~source_topic", "/carla/hero/LIDAR"),
            PointCloud2,
            self.callback,
        )

        rospy.spin()

    def reconstruct_img_from_lidar(self, coordinates_xyz, focus):
        """
        reconstruct 3d LIDAR-Data and calculate 2D Pixel
        according to Camera-World

        Args:
            coordinates_xyz (np.array): filtered lidar points
            focus (String): Camera Angle

        Returns:
            image: depth image for camera angle
        """

        # intrinsic matrix for camera:
        # width -> 300, height -> 200, fov -> 100 (agent.py)
        im = np.identity(3)
        im[0, 2] = 1280 / 2.0
        im[1, 2] = 720 / 2.0
        im[0, 0] = im[1, 1] = 1280 / (2.0 * np.tan(100 * np.pi / 360.0))

        # extrinsic matrix for camera
        ex = np.zeros(shape=(3, 4))
        ex[0][0] = 1
        ex[1][1] = 1
        ex[2][2] = 1
        m = np.matmul(im, ex)

        # reconstruct camera image with LIDAR-Data
        img = np.zeros(shape=(720, 1280), dtype=np.float32)
        dist_array = np.zeros(shape=(720, 1280, 3), dtype=np.float32)
        for c in coordinates_xyz:
            # center depth image
            if focus == "Center":
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y <= 720:
                    img[719 - y][1279 - x] = c[0]
                    dist_array[719 - y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            # back depth image
            if focus == "Back":
                point = np.array([c[1], c[2], c[0], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y < 720:
                    img[y][1279 - x] = -c[0]
                    dist_array[y][1279 - x] = np.array(
                        [-c[0], c[1], c[2]], dtype=np.float32
                    )

            # left depth image
            if focus == "Left":
                point = np.array([c[0], c[2], c[1], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x <= 1280 and y >= 0 and y <= 720:
                    img[719 - y][1279 - x] = c[1]
                    dist_array[y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

            # right depth image
            if focus == "Right":
                point = np.array([c[0], c[2], c[1], 1])
                pixel = np.matmul(m, point)
                x, y = int(pixel[0] / pixel[2]), int(pixel[1] / pixel[2])
                if x >= 0 and x < 1280 and y >= 0 and y < 720:
                    img[y][1279 - x] = -c[1]
                    dist_array[y][1279 - x] = np.array(
                        [c[0], c[1], c[2]], dtype=np.float32
                    )

        return dist_array


def ros_init():
    """Initializes the node for basic ROS functions.

    Must only be called ONCE and not as part of def main()

    Required for debugger entry"""
    rospy.init_node("lidar_distance")


def main():
    """Main entry point of this node

    Required for debugger entry"""
    lidar_distance = LidarDistance()
    lidar_distance.listener()


if __name__ == "__main__":
    rospy.init_node("lidar_distance")
    main()

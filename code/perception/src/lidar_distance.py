#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2
from perception.msg import MinDistance

from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from itertools import combinations
import cv2
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge


class LidarDistance():
    """ See doc/06_perception/03_lidar_distance_utility.md on
        how to configute this node
    """

    def callback(self, data):
        """ Callback function, filters a PontCloud2 message
            by restrictions defined in the launchfile.

            Publishes a range message containing the farest and
            the closest point of the filtered result. Additionally,
            publishes the filtered result as PointCloud2
            on the topic defined in the launchfile.

        :param data: a PointCloud2
        """
        coordinates = ros_numpy.point_cloud2.pointcloud2_to_array(data)

        # https://stackoverflow.com/questions/44295375/how-to-slice-a-numpy-
        # ndarray-made-up-of-numpy-void-numbers
        bit_mask = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=rospy.get_param('~max_x', np.inf),
            min_x=rospy.get_param('~min_x', -np.inf),
            max_y=rospy.get_param('~max_y', np.inf),
            min_y=rospy.get_param('~min_y', -np.inf),
            max_z=rospy.get_param('~max_z', np.inf),
            min_z=rospy.get_param('~min_z', -np.inf),
        )

        # Filter coordinates based in generated bit_mask
        coordinates = coordinates[bit_mask]

        # Create pointcloud from manipulated data
        coordinates_manipulated = ros_numpy \
            .point_cloud2.array_to_pointcloud2(coordinates)
        coordinates_manipulated.header = data.header

        # Publish manipulated pointCloud2
        self.pub_pointcloud.publish(coordinates_manipulated)

        # https://stackoverflow.com/questions/1401712/how-can-the-euclidean-
        # distance-be-calculated-with-numpy
        coordinates_xyz = np.array(
            lidar_filter_utility.remove_field_name(coordinates, 'intensity')
            .tolist()
        )
        plot = self.plot_blob(coordinates_xyz)
        img_msg = self.bridge.cv2_to_imgmsg(plot,
                                            encoding="passthrough")
        img_msg.header = data.header
        self.publisher.publish(img_msg)

        """
        distances = np.array(
            [np.linalg.norm(c - [0, 0, 0]) for c in coordinates_xyz])

        if len(distances) > 0:
            range_msg = Range()
            range_msg.max_range = max(distances)
            range_msg.min_range = min(distances)
            range_msg.range = min(distances)

            self.pub_range.publish(range_msg)"""

    def listener(self):
        """ Initializes the node and it's publishers
        """
        # run simultaneously.
        rospy.init_node('lidar_distance')

        self.pub_pointcloud = rospy.Publisher(
            rospy.get_param(
                '~point_cloud_topic',
                '/carla/hero/' + rospy.get_namespace() + '_filtered'
            ),
            PointCloud2
        )

        self.pub_range = rospy.Publisher(
            rospy.get_param(
                '~range_topic',
                '/carla/hero/' + rospy.get_namespace() + '_min_distance'
            ),
            MinDistance
        )

        self.publisher = rospy.Publisher(
            rospy.get_param(
                '~image_distance_topic',
                '/paf/hero/Center/segmented_image'
            ),
            ImageMsg
        )

        self.bridge = CvBridge()

        rospy.Subscriber(rospy.get_param('~source_topic', "/carla/hero/LIDAR"),
                         PointCloud2, self.callback)

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()

    def plot_blob(self, xyz_coords):
        # Standardize the data
        xyz_coords_standardized = StandardScaler().fit_transform(xyz_coords)

        # Compute pairwise distances in the x-axis
        pairwise_distances_x = np.abs(np.subtract.outer(xyz_coords[:, 0],
                                                        xyz_coords[:, 0]))

        # Find the absolute minimum x distance
        min_x_distance = np.min(pairwise_distances_x[pairwise_distances_x > 0])

        # print(f"Absolute minimum x
        # distance: {min_x_distance:.4f} meters")
        self.pub_range.publish(min_x_distance)

        # Apply DBSCAN algorithm
        # Maximum distance between two samples for one to be
        # considered as in the neighborhood of the other
        eps = 0.2

        # The number of samples in a neighborhood for a point
        # to be considered as a core point
        min_samples = 2
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(xyz_coords_standardized)

        # Plot the clustered points
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for label in set(labels):
            if label == -1:  # Noise points
                ax.scatter(xyz_coords[labels == label][:, 0],
                           xyz_coords[labels == label][:, 1],
                           xyz_coords[labels == label][:, 2],
                           c='gray', marker='o', label='Noise')
            else:
                ax.scatter(xyz_coords[labels == label][:, 0],
                           xyz_coords[labels == label][:, 1],
                           xyz_coords[labels == label][:, 2],
                           label=f'Cluster {label + 1}', s=50)

        # Set axis labels
        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        fig.canvas.draw()

        # convert canvas to image
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))

        # img is rgb, convert to opencv's default bgr
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img


if __name__ == '__main__':
    lidar_distance = LidarDistance()
    lidar_distance.listener()

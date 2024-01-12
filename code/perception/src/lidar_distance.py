#!/usr/bin/env python
import rospy
import ros_numpy
import numpy as np
import lidar_filter_utility
from sensor_msgs.msg import PointCloud2
# from perception.msg import MinDistance
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
import matplotlib.pyplot as plt
# from mpl_toolkits.mplot3d import Axes3D
# from itertools import combinations
import cv2
from sensor_msgs.msg import Image as ImageMsg
from cv_bridge import CvBridge
# from matplotlib.colors import LinearSegmentedColormap


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
        min_dist_bit_mask = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=rospy.get_param('~max_x', np.inf),
            min_x=rospy.get_param('~min_x', -np.inf),
            min_z=rospy.get_param('~min_z', -np.inf),
            min_y=rospy.get_param('~min_y', -np.inf),
            max_y=rospy.get_param('~max_y', np.inf),
        )

        reconstruct_bit_mask = lidar_filter_utility.bounding_box(
            coordinates,
            max_x=rospy.get_param('~max_x', np.inf),
            min_x=rospy.get_param('~min_x', -np.inf),
            min_z=rospy.get_param('~min_z', -np.inf),
        )

        # Filter coordinates based in generated bit_mask
        min_dist_coordinates = coordinates[min_dist_bit_mask]
        reconstruct_coordinates = coordinates[reconstruct_bit_mask]

        # Create pointcloud from manipulated data
        coordinates_manipulated = ros_numpy \
            .point_cloud2.array_to_pointcloud2(min_dist_coordinates)
        coordinates_manipulated.header = data.header

        # Publish manipulated pointCloud2
        self.pub_pointcloud.publish(coordinates_manipulated)

        # https://stackoverflow.com/questions/1401712/how-can-the-euclidean-
        # distance-be-calculated-with-numpy
        min_dist_coordinates_xyz = np.array(
            lidar_filter_utility.remove_field_name(min_dist_coordinates,
                                                   'intensity')
            .tolist()
        )

        reconstruct_coordinates_xyz = np.array(
            lidar_filter_utility.remove_field_name(reconstruct_coordinates,
                                                   'intensity')
            .tolist()
        )

        # handle minimum distance
        if min_dist_coordinates_xyz.shape[0] > 0:
            plot = self.plot_blob(min_dist_coordinates_xyz)
            img_msg = self.bridge.cv2_to_imgmsg(plot,
                                                encoding="passthrough")
            img_msg.header = data.header
            self.min_dist_img_publisher.publish(img_msg)
        # else:
            # self.pub_min_dist.publish(np.inf)

        # handle reconstruction of lidar points
        rainbow_cloud = self.reconstruct_img_from_lidar(
            reconstruct_coordinates_xyz)

        img_msg = self.bridge.cv2_to_imgmsg(rainbow_cloud,
                                            encoding="passthrough")
        img_msg.header = data.header
        self.rainbow_publisher.publish(img_msg)

    def listener(self):
        """ Initializes the node and it's publishers
        """
        # run simultaneously.
        rospy.init_node('lidar_distance')
        self.bridge = CvBridge()

        self.pub_pointcloud = rospy.Publisher(
            rospy.get_param(
                '~point_cloud_topic',
                '/carla/hero/' + rospy.get_namespace() + '_filtered'
            ),
            PointCloud2,
            queue_size=10
        )

        # publisher for the closest blob in the lidar point cloud
        """self.pub_min_dist = rospy.Publisher(
            rospy.get_param(
                '~range_topic',
                '/paf/hero/Center/min_distance'
            ),
            MinDistance,
            queue_size=10
        )
        """
        # publisher for reconstructed lidar image
        self.rainbow_publisher = rospy.Publisher(
            rospy.get_param(
                '~image_distance_topic',
                '/paf/hero/Center/rainbow_image'
            ),
            ImageMsg,
            queue_size=10
        )

        # publisher for 3d blob graph
        self.min_dist_img_publisher = rospy.Publisher(
            rospy.get_param(
                '~image_distance_topic',
                '/paf/hero/Center/min_dist_image'
            ),
            ImageMsg,
            queue_size=10
        )

        rospy.Subscriber(rospy.get_param('~source_topic', "/carla/hero/LIDAR"),
                         PointCloud2, self.callback)

        rospy.spin()

    def plot_blob(self, xyz_coords):
        # creates a 3d graph thazt highlights blobs of points
        xyz_coords_standardized = StandardScaler().fit_transform(xyz_coords)
        # pairwise_distances_x = np.abs(np.subtract.outer(xyz_coords[:, 0],
        # xyz_coords[:, 0]))

        # publish minimum distance
        # min_x_distance = np.min(p
        # airwise_distances_x[pairwise_distances_x > 0])

        eps = 0.2
        min_samples = 5
        dbscan = DBSCAN(eps=eps, min_samples=min_samples)
        labels = dbscan.fit_predict(xyz_coords_standardized)

        min_distances_within_clusters = []

        # Iterate through each cluster
        for label in set(labels):
            if label != -1:  # Ignore noise points
                cluster_points = xyz_coords[labels == label]
                pairwise_distances = np.linalg.norm(
                    cluster_points -
                    np.mean(cluster_points, axis=0), axis=1)
                min_distance_within_cluster = np.min(
                    pairwise_distances)
                min_distances_within_clusters.append(
                    min_distance_within_cluster)

        # Find the overall minimum distance within clusters
        # min_distance_within_clusters = np.min(min_distances_within_clusters)

        # Publish the minimum distance within clusters
        # self.pub_min_dist.publish(min_distance_within_clusters)

        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')

        for label in set(labels):
            # print(label)
            if label == -1:
                ax.scatter(xyz_coords[labels == label][:, 0],
                           xyz_coords[labels == label][:, 1],
                           xyz_coords[labels == label][:, 2],
                           c='gray', marker='o', label='Noise')
            else:
                ax.scatter(xyz_coords[labels == label][:, 0],
                           xyz_coords[labels == label][:, 1],
                           xyz_coords[labels == label][:, 2],
                           label=f'Cluster {label + 1}', s=50)

        ax.set_xlabel('X')
        ax.set_ylabel('Y')
        ax.set_zlabel('Z')

        fig.canvas.draw()
        img = np.fromstring(fig.canvas.tostring_rgb(), dtype=np.uint8, sep='')
        img = img.reshape(fig.canvas.get_width_height()[::-1] + (3,))
        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        return img

    def reconstruct_img_from_lidar(self, coordinates_xyz):
        # reconstruct 3d LIDAR-Data and calculate 2D Pixel
        # according to Camera-World

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
        for c in coordinates_xyz:
            point = np.array([c[1], c[2], c[0], 1])
            pixel = np.matmul(m, point)
            x, y = int(pixel[0]/pixel[2]), int(pixel[1]/pixel[2])
            if x >= 0 and x <= 1280 and y >= 0 and y <= 720:
                img[719-y][1279-x] = c[0]

        # Rainbox color mapping to highlight distances
        """colors = [(0, 0, 0)] + [(1, 0, 0), (1, 1, 0),
                                (0, 1, 0), (0, 1, 1),
                                (0, 0, 1)]
        cmap_name = 'rainbow'
        rainbow_cmap = LinearSegmentedColormap.from_list(cmap_name,
                                                         colors,
                                                         N=256)

        img_colored = (rainbow_cmap(img / np.max(img)) * 255).astype(np.uint8)
        img_bgr = cv2.cvtColor(img_colored, cv2.COLOR_RGBA2BGR)"""

        return img


if __name__ == '__main__':
    lidar_distance = LidarDistance()
    lidar_distance.listener()

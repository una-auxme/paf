#!/usr/bin/env python
# ROS imports
import ros_compatibility as roscomp
from ros_compatibility.node import CompatibleNode
from sensor_msgs.msg import Image as ImageMsg
from rospy.numpy_msg import numpy_msg
from cv_bridge import CvBridge
from scipy.spatial.transform import Rotation as R
import numpy as np


class lane_position(CompatibleNode):
    def __init__(self, name, **kwargs):
        super().__init__(name, **kwargs)
        self.bridge = CvBridge()
        # get camera parameters
        self.camera_x = self.get_param("camera_x")
        self.camera_y = self.get_param("camera_y")
        self.camera_z = self.get_param("camera_z")
        self.camera_roll = self.get_param("camera_roll")
        self.camera_yaw = self.get_param("camera_yaw")
        self.camera_pitch = self.get_param("camera_pitch")
        self.camera_width = self.get_param("camera_width")
        self.camera_height = self.get_param("camera_height")
        self.camera_fov = self.get_param("camera_fov")
        self.create_Matrices()

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
        lanemask = self.bridge.imgmsg_to_cv2(img_msg=ImageMsg, desired_encoding="8UC1")
        world_coordinates = self.transform_camera2worldcoordinates(lanemask)
        pass

    def create_Matrices(self):
        f_x = f_y = self.camera_width / (2 * np.tan(np.radians(self.camera_fov) / 2))
        c_x = self.camera_width / 2
        c_y = self.camera_height / 2
        # Interne Kameramatrix
        self.K = np.array([[f_x, 0, c_x], [0, f_y, c_y], [0, 0, 1]])
        self.R = self.create_rotation_matrix(
            self.camera_roll, self.camera_pitch, self.camera_yaw
        )
        self.T = np.array([self.camera_x, self.camera_y, self.camera_z])
        self.K_inv = np.linalg.inv(self.K)
        # Create a grid of pixel coordinates (u, v)
        self.u, self.v = np.meshgrid(
            np.arange(self.camera_width), np.arange(self.camera_height)
        )

    def transform_camera2worldcoordinates(self, lanemask):

        # Flatten the mask and pixel coordinates
        mask_flat = lanemask.flatten()
        u_flat = self.u.flatten()
        v_flat = self.v.flatten()
        # Select only the masked pixels
        masked_indices = np.where(mask_flat == 255)[0]
        u_selected = u_flat[masked_indices]
        v_selected = v_flat[masked_indices]
        # Construct pixel coordinates in homogeneous form
        pixel_coords = np.stack(
            [u_selected, v_selected, np.ones_like(u_selected)], axis=0
        )
        # Transform pixel coordinates to normalized camera coordinates
        norm_camera_coords = self.K_inv @ pixel_coords
        # Project to the ground plane (z_world = 0)
        z_c = self.T[2]  # Camera height
        camera_coords = norm_camera_coords * z_c

        # Transform to world coordinates
        world_coords_homogeneous = np.dot(
            self.R.T, camera_coords - self.T[:, np.newaxis]
        )

        # Extract X, Y world coordinates
        world_coords = world_coords_homogeneous[:2, :].T
        return world_coords

    def create_rotation_matrix(self, roll, pitch, yaw):
        """
        Computes the rotation matrix from roll, pitch, and yaw angles.
        :param roll: Rotation around the x-axis (in degrees)
        :param pitch: Rotation around the y-axis (in degrees)
        :param yaw: Rotation around the z-axis (in degrees)
        :return: 3x3 rotation matrix
        """
        # Convert angles from degrees to radians
        roll = np.radians(roll)
        pitch = np.radians(pitch)
        yaw = np.radians(yaw)

        # Rotation matrix for rotation around the x-axis (Roll)
        R_z = np.array(
            [
                [1, 0, 0],
                [0, np.cos(yaw), -np.sin(yaw)],
                [0, np.sin(yaw), np.cos(yaw)],
            ]
        )

        # Rotation matrix for rotation around the y-axis (Pitch)
        R_y = np.array(
            [
                [np.cos(pitch), 0, np.sin(pitch)],
                [0, 1, 0],
                [-np.sin(pitch), 0, np.cos(pitch)],
            ]
        )

        # Rotation matrix for rotation around the z-axis (Yaw)
        R_x = np.array(
            [
                [np.cos(roll), -np.sin(roll), 0],
                [np.sin(roll), np.cos(roll), 0],
                [0, 0, 1],
            ]
        )

        # Combined rotation matrix (R = Rz * Ry * Rx)
        R = R_z @ R_y @ R_x
        return R


if __name__ == "__main__":
    roscomp.init("Lanedetection_node")
    node = lane_position("Lanedetection_node")
    node.run()

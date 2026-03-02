from mapping_interfaces.msg import ClusteredPointsArray
from geometry_msgs.msg import PoseStamped

from rclpy.time import Time
from scipy.spatial.transform import Rotation

import numpy as np
from tf_transformations import quaternion_matrix


def array_to_clustered_points(
    stamp: Time,
    points,
    point_indices,
    object_speed_array=None,
    object_class_array=None,
    header_id="hero",
):
    """
    Convert the given points and point indices to a ClusteredPointsArray message.

    Args:
        points: numpy array with shape (N, 3)
        point_indices: numpy array with the shape (N,)
        object_speed_array: numpy array with the shape (N,)
        object_class_array: numpy array with the shape (N,)
        header_id: string

    Returns:
        ClusteredPointsArray message
    """
    # Create the ClusteredPointsArray message
    clustered_points = ClusteredPointsArray()
    clustered_points.header.frame_id = header_id
    clustered_points.header.stamp = stamp.to_msg()

    # Flatten the points array into a single list of [x1, y1, z1, x2, y2, z2, ...]
    clustered_points.cluster_points_array = points.flatten().tolist()

    # Populate the indexArray
    clustered_points.index_array = point_indices.astype(int).tolist()

    # Populate the motionArray if object_speed_array is provided
    if object_speed_array is not None:
        clustered_points.motion_array = object_speed_array
        # rospy.logerr("Motion2D is not implemented")

    # Populate the object_class if object_class_array is provided
    if object_class_array is not None:
        clustered_points.object_class = object_class_array

    return clustered_points


def ego_motion_compensation(points: np.ndarray, dT: np.ndarray) -> np.ndarray:
    """
    Applies the relative motion transformation matrix (dT) to a point cloud.

    The point cloud is converted to homogeneous coordinates, multiplied by the
    4x4 transformation matrix, and then converted back to XYZ coordinates.

    :param points: Structured NumPy array of points (must contain 'x', 'y', 'z' fields).
    :param dT: The 4x4 homogeneous transformation matrix representing the
               relative motion (T_prev @ inv(T_cur)).
    :return: Structured NumPy array of the compensated points.
    """

    comp_points = np.copy(points)
    points_xyz = np.stack(
        [comp_points["x"], comp_points["y"], comp_points["z"]], axis=1
    )
    N = points_xyz.shape[0]

    homogeneous_points = np.hstack([points_xyz, np.ones((N, 1))])
    P = homogeneous_points.T

    transformed_points = dT @ P
    transformed_points_xyz = transformed_points[:3, :]

    comp_points["x"] = transformed_points_xyz[0, :]
    comp_points["y"] = transformed_points_xyz[1, :]
    comp_points["z"] = transformed_points_xyz[2, :]

    return comp_points


def create_delta_matrix(cur_pos: PoseStamped, prev_pos: PoseStamped) -> np.ndarray:
    """
    Creates the relative homogeneous transformation matrix (dT) between two poses.
    This matrix moves points from the current frame's perspective back to the
    previous frame's perspective.

    :param cur_pos: PoseStamped object for the current vehicle position (T_cur).
    :param prev_pos: PoseStamped object for the previous vehicle position (T_prev).
    :return: The 4x4 relative motion transformation matrix (dT).
    """

    T_cur = create_transform_matrix(cur_pos)
    T_prev = create_transform_matrix(prev_pos)

    return T_prev @ np.linalg.inv(T_cur)


def create_transform_matrix(pos: PoseStamped) -> np.ndarray:
    """
    Converts a PoseStamped object into a 4x4 homogeneous transformation matrix (T).

    T = [ R | t ]
        [ 0 | 1 ]

    :param pos: PoseStamped object containing position (t) and orientation (q).
    :return: The 4x4 homogeneous transformation matrix (T).
    """

    t = pos.pose.position
    q = pos.pose.orientation

    translation = (t.x, t.y, t.z)
    quaternion = (q.x, q.y, q.z, q.w)

    R = quaternion_matrix(quaternion)

    T = np.copy(R)
    T[:3, 3] = translation

    return T


def create_ego_vehicle_mask(data_array: np.ndarray) -> np.ndarray:
    """
    Creates a boolean mask to identify points belonging to the ego vehicle structure.

    The mask defines a simple rectangular region in the vehicle's local frame
    (centered around the vehicle body).

    :param data_array: Structured NumPy array of points
                       (must contain 'x' and 'y' fields).
    :return: Boolean NumPy array (mask) where True indicates an ego vehicle point.
    """

    min_x = -2
    max_x = 2
    min_y = -1
    max_y = 1

    mask_x = (data_array["x"] >= min_x) & (data_array["x"] <= max_x)
    mask_y = (data_array["y"] >= min_y) & (data_array["y"] <= max_y)

    ego_mask = mask_x & mask_y
    return ego_mask


def apply_local_motion_compensation(
    points: np.ndarray, d_x: float, d_heading: float, account_heading: bool = True
) -> np.ndarray:
    """
    Applies a simple 2D motion correction to static points in the vehicle's local frame.

    1. Translates points back along the X (forward) axis by d_x.
    2. Optionally rotates the points by d_heading (rotation correction).

    :param points: Structured NumPy array of static environment points.
    :param d_x: The distance the vehicle traveled forward (translation component).
    :param d_heading: The change in yaw angle (heading) of the vehicle.
    :param account_heading: If True, rotation compensation is applied.
    :return: Structured NumPy array of the compensated static points.
    """

    comp_points = np.copy(points)
    comp_points["x"] = comp_points["x"] - d_x

    if account_heading:
        points_xyz = np.stack([comp_points["x"], comp_points["y"], comp_points["z"]])
        R = Rotation.from_euler("z", d_heading, degrees=True).as_matrix()
        comp_3xN = R @ points_xyz

        comp_points["x"] = comp_3xN[0, :]
        comp_points["y"] = comp_3xN[1, :]
        comp_points["z"] = comp_3xN[2, :]

    return comp_points


def quaternion_to_heading(x: float, y: float, z: float, w: float) -> float:
    """
    Translates quaternion to euler heading.
    :param x:
    :param y:
    :param z:
    :param w:
    :return: euler heading of the given quaternion
    """

    quaternion = (x, y, z, w)
    rot = Rotation.from_quat(quaternion)
    rot_euler = rot.as_euler("xyz", degrees=True)
    return rot_euler[2]

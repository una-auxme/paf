import numpy as np


# https://gist.github.com/bigsnarfdude/bbfdf343cc2fc818dc08b58c0e1374ae
def bounding_box(
    points,
    min_x=-np.inf,
    max_x=np.inf,
    min_y=-np.inf,
    max_y=np.inf,
    min_z=-np.inf,
    max_z=np.inf,
):
    """Compute a bounding_box filter on the given points

    Parameters
    ----------
    points: (n,3) array
        The array containing all the points's coordinates.
        Expected format:
            array([
                [x1,y1,z1],
                ...,
                [xn,yn,zn]])

    min_i, max_i: float
        The bounding box limits for each coordinate.
        If some limits are missing, the default values
        are -infinite for the min_i and infinite for the max_i.

    Returns
    -------
    bb_filter : boolean array
        The boolean mask indicating wherever a point should be
        keeped or not. The size of the boolean mask will be the
        same as the number of given points.

    """

    bound_x = np.logical_and(points["x"] > min_x, points["x"] < max_x)
    bound_y = np.logical_and(points["y"] > min_y, points["y"] < max_y)
    bound_z = np.logical_and(points["z"] > min_z, points["z"] < max_z)

    bb_filter = bound_x & bound_y & bound_z

    return bb_filter


def filter_ground_points(
    points,
    z_min,
    z_max,
    pitch_rad=0.0,
    enable_pitch_compensation=True,
):
    """Filter lidar points using a base height window and an optional pitch-aware
    ground plane.

    The existing static cutoff is preserved when pitch compensation is disabled or
    when ``pitch_rad`` is zero. When enabled, the lower z-bound is tilted along the
    x-axis so that forward road points do not drift into the obstacle set during
    ego pitch motion.

    Parameters
    ----------
    points:
        Structured numpy array with ``x`` and ``z`` fields.
    z_min, z_max:
        Base vertical bounds for obstacle clustering.
    pitch_rad:
        Current ego pitch in radians.
    enable_pitch_compensation:
        Whether to tilt the lower bound with the current pitch.

    Returns
    -------
    numpy.ndarray
        Filtered structured array containing only points inside the valid region.
    """

    if points.size == 0:
        return points

    lower_bound = np.full(points.shape[0], z_min, dtype=float)
    if enable_pitch_compensation:
        lower_bound = lower_bound + points["x"] * np.tan(pitch_rad)

    mask = (points["z"] > lower_bound) & (points["z"] < z_max)
    return points[mask]


# https://stackoverflow.com/questions/15575878/how-do-you-remove-a-column-from-a-structured-numpy-array
def remove_field_name(a, name):
    """Removes a column from a structured numpy array

    :param a: structured numoy array
    :param name: name of the column to remove
    :return: structured numpy array without column
    """
    names = list(a.dtype.names)
    if name in names:
        names.remove(name)
    b = a[names]
    return b

#!/usr/bin/env python
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


# https://stackoverflow.com/questions/15575878/how-do-you-remove-a-column-from-a-structured-numpy-array
def remove_field_name(a, name):
    """Removes a column from a structured numpy array

    :param a: structured numpy array
    :param name: name of the column to remove
    :return: structured numpy array without column
    """
    names = list(a.dtype.names)
    if name in names:
        names.remove(name)
    b = a[names]
    return b

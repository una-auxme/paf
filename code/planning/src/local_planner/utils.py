from scipy.spatial.transform import Rotation
import numpy as np
import math

hyperparameters = {
    "max_speed": 15,
    "max_accel": 4.0,
    "max_curvature": 30.0,
    "max_road_width_l": 0.0,
    "max_road_width_r": 3.5,
    "d_road_w": 0.2,
    "dt": 0.2,
    "maxt": 17.0,
    "mint": 6.0,
    "d_t_s": 0.5,
    "n_s_sample": 2.0,
    "obstacle_clearance": 1,
    "kd": 1.0,
    "kv": 0.1,
    "ka": 0.1,
    "kj": 0.1,
    "kt": 0.1,
    "ko": 0.1,
    "klat": 1.0,
    "klon": 1.0,
    "num_threads": 1,  # set 0 to avoid using threaded algorithm
}


def location_to_gps(lat_ref, lon_ref, x, y):
    """
    Convert from world coordinates to GPS coordinates
    :param lat_ref: latitude reference for the current map
    :param lon_ref: longitude reference for the current map
    :param location: location to translate
    :return: dictionary with lat, lon and height
    """

    EARTH_RADIUS_EQUA = 6378137.0   # pylint: disable=invalid-name
    scale = math.cos(lat_ref * math.pi / 180.0)
    mx = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
    my = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat_ref) *
                                                       math.pi / 360.0))
    mx += x
    my -= y

    lon = mx * 180.0 / (math.pi * EARTH_RADIUS_EQUA * scale)
    lat = 360.0 * math.atan(math.exp(my / (EARTH_RADIUS_EQUA * scale))) / math.pi - 90.0
    z = 703

    return {'lat': lat, 'lon': lon, 'z': z}


def approx_obstacle_pos(distance: float, heading: float,
                        ego_pos: np.array, speed: float):
    """calculate the position of the obstacle in the global coordinate system
        based on ego position, heading and distance

    Args:
        speed (float): Speed of the ego vehicle
        distance (float): Distance to the obstacle
        heading (float): Ego vehivle heading
        ego_pos (np.array): Position in [x, y, z]

    Returns:
        np.array: approximated position of the obstacle
    """
    rotation_matrix = Rotation.from_euler('z', heading)

    # Create distance vector with 0 rotation
    relative_position_local = np.array([distance, 0, 0])
    
    # speed vector
    speed_vector = rotation_matrix.apply(np.array([speed, 0, 0]))
    # Rotate distance vector to match heading
    absolute_position_local = rotation_matrix.apply(relative_position_local)

    # Add egomposition vector with distance vetor to get absolute position
    vehicle_position_global_start = ego_pos + absolute_position_local

    length = np.array([3, 0, 0])
    length_vector = rotation_matrix.apply(length)

    offset = np.array([0.75, 0, 0])
    rotation_adjusted = Rotation.from_euler('z', heading + math.radians(90))
    offset_front = rotation_adjusted.apply(offset)

    rotation_adjusted = Rotation.from_euler('z', heading + math.radians(270))
    offset_back = rotation_adjusted.apply(offset)

    vehicle_position_global_end = vehicle_position_global_start + \
        length_vector + offset_back

    return vehicle_position_global_start + offset_front, \
        vehicle_position_global_end, speed_vector


def convert_to_ms(speed):
    return speed / 3.6

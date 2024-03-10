from scipy.spatial.transform import Rotation
import numpy as np
import math
# import rospy


hyperparameters = {
    "max_speed": 15,
    "max_accel": 4.0,
    "max_curvature": 30.0,
    "max_road_width_l": 4,
    "max_road_width_r": 4,
    "d_road_w": 0.2,
    "dt": 0.2,
    "maxt": 30,
    "mint": 6.0,
    "d_t_s": 0.5,
    "n_s_sample": 2.0,
    "obstacle_clearance": 1.5,
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


def location_to_gps(lat_ref: float, lon_ref: float, x: float, y: float):
    """Convert world coordinates to (lat,lon,z) coordinates
       Copied from:
       https://github.com/carla-simulator/scenario_runner/blob/master/srunner/tools/route_manipulation.py

    Args:
        lat_ref (float): reference lat value
        lon_ref (float): reference lat value
        x (float): x-Coordinate value
        y (float): y-Coordinate value

    Returns:
        dict: Dictionary with (lat,lon,z) coordinates
    """

    EARTH_RADIUS_EQUA = 6378137.0   # pylint: disable=invalid-name
    scale = math.cos(lat_ref * math.pi / 180.0)
    mx = scale * lon_ref * math.pi * EARTH_RADIUS_EQUA / 180.0
    my = scale * EARTH_RADIUS_EQUA * math.log(math.tan((90.0 + lat_ref) *
                                                       math.pi / 360.0))
    mx += x
    my -= y

    lon = mx * 180.0 / (math.pi * EARTH_RADIUS_EQUA * scale)
    lat = 360.0 * math.atan(math.exp(my / (EARTH_RADIUS_EQUA * scale))) /\
        math.pi - 90.0
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

    offset = np.array([1, 0, 0])
    rotation_adjusted = Rotation.from_euler('z', heading + math.radians(90))
    offset_front = rotation_adjusted.apply(offset)

    rotation_adjusted = Rotation.from_euler('z', heading + math.radians(270))
    offset_back = rotation_adjusted.apply(offset)

    vehicle_position_global_end = vehicle_position_global_start + \
        length_vector + offset_back

    return vehicle_position_global_start + offset_front, \
        vehicle_position_global_end, speed_vector


def convert_to_ms(speed: float):
    """Convert km/h to m/s

    Args:
        speed (float): speed in km/h

    Returns:
        float: speed in m/s
    """
    return speed / 3.6


def spawn_car(distance):
    """Only used for testing, spawns a car in the given distance

    Args:
        distance (float): distance
    """
    import carla
    import os
    CARLA_HOST = os.environ.get('CARLA_HOST', 'paf23-carla-simulator-1')
    CARLA_PORT = int(os.environ.get('CARLA_PORT', '2000'))

    client = carla.Client(CARLA_HOST, CARLA_PORT)

    world = client.get_world()
    world.wait_for_tick()

    blueprint_library = world.get_blueprint_library()
    # bp = blueprint_library.filter('vehicle.*')[0]
    # vehicle = world.spawn_actor(bp, world.get_map().get_spawn_points()[0])
    bp = blueprint_library.filter("model3")[0]
    for actor in world.get_actors():
        if actor.attributes.get('role_name') == "hero":
            ego_vehicle = actor
            break

    spawnPoint = carla.Transform(ego_vehicle.get_location() +
                                 carla.Location(y=distance.data),
                                 ego_vehicle.get_transform().rotation)
    vehicle = world.spawn_actor(bp, spawnPoint)

    vehicle.set_autopilot(False)
    # vehicle.set_location(loc)
    # coords = vehicle.get_location()
    # get spectator
    spectator = world.get_spectator()
    # set spectator to follow ego vehicle with offset
    spectator.set_transform(
        carla.Transform(ego_vehicle.get_location() + carla.Location(z=50),
                        carla.Rotation(pitch=-90)))


def filter_vision_objects(float_array):
    """Filters vision objects to calculate collision check
    It contains the classId, the absolute Euclidean distance
    and 6 coordinates for upper left and lower right corner
    of the bounding box

    Array shape: [classID, EuclidDistance,
                    UpperLeft(x,y,z), LowerRight(x,y,z)]

    Args:
        data (ndarray): numpy array with vision objects
    """

    """
    LEON:

    Ihr bekommt jetzt nur 3-Werte -> ClassIndex, Min_X, Min_Abs_Y

    Min_Abs_Y ist der nähste Punkt vom Object zu Y=0 also der Mitte.

    Damit habt ihr immer automatisch den
    nähesten und wichtigsten Punkt des Objekts.

    """

    # Reshape array to 3 columns and n rows (one row per object)
    float_array = np.asarray(float_array)
    float_array = np.reshape(float_array, (float_array.size//3, 3))
    # Filter all rows that contain np.inf
    float_array = float_array[~np.any(np.isinf(float_array), axis=1), :]
    if float_array.size == 0:
        return None
    # Filter out all objects that are not cars
    all_cars = float_array[np.where(float_array[:, 0] == 2)]

    # Get cars that are on our lane
    cars_in_front = all_cars[np.where(np.abs(all_cars[:, 2]) < 0.3)]
    if cars_in_front.size == 0:
        # no car in front
        return None
    # Filter for potential recognition of ega vehicle front hood
    filtered_cars_in_front = cars_in_front[np.where(cars_in_front[:, 1] > 0.7)]
    if filtered_cars_in_front.size == 0:
        # no car in front
        return None
    # Return nearest car
    return filtered_cars_in_front[np.argmin(filtered_cars_in_front[:, 1])]

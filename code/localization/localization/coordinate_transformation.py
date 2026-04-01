"""
This script provides coordinate transformations from Geodetic -> ECEF,
ECEF -> ENU and Geodetic -> ENU
(the composition of the two previous functions).
Running the script by itself runs tests.
based on https://gist.github.com/govert/1b373696c9a27ff4c72a.
A good source to read up on the different reference frames is:
http://dirsig.cis.rit.edu/docs/new/coordinates.html
"""

import math
from xml.etree import ElementTree as eTree

import numpy as np
import pymap3d
import pymap3d.ellipsoid
from scipy.spatial.transform import Rotation

try:
    from pyproj import CRS, Transformer
except ModuleNotFoundError:
    CRS = None
    Transformer = None


a = 6378137  # EARTH_RADIUS_EQUA in Pylot, used in geodetic_to_enu
b = 6356752.3142
f = (a - b) / a
e_sq = f * (2 - f)
alt_offset = 331.00000
STD_LAT = 0.0
STD_LON = 0.0
STD_H = 0.0
_GEO_REFERENCE_SKIP_PREFIXES = (
    "+geoidgrids=",
    "+vunits=",
)


def extract_geo_reference_from_opendrive(opendrive: str) -> str:
    """Return the OpenDRIVE geoReference string used by CARLA for GNSS projection."""
    root = eTree.fromstring(opendrive)
    header = root.find("header")
    if header is None:
        raise ValueError("OpenDRIVE header missing.")

    geo_reference = header.findtext("geoReference")
    if geo_reference is None:
        raise ValueError("OpenDRIVE geoReference missing.")

    return geo_reference.strip()


def _extract_proj_parameter(
    geo_reference: str, parameter: str, default: float
) -> float:
    index = geo_reference.find(parameter)
    if index == -1:
        return default

    end_index = geo_reference.find(" ", index)
    if end_index == -1:
        end_index = len(geo_reference)

    return float(geo_reference[index + len(parameter) : end_index])


def _extract_proj_string_parameter(
    geo_reference: str, parameter: str, default: str
) -> str:
    index = geo_reference.find(parameter)
    if index == -1:
        return default

    end_index = geo_reference.find(" ", index)
    if end_index == -1:
        end_index = len(geo_reference)

    return geo_reference[index + len(parameter) : end_index]


def _sanitize_geo_reference(geo_reference: str) -> str:
    return " ".join(
        token
        for token in geo_reference.replace("\n", " ").split()
        if not token.startswith(_GEO_REFERENCE_SKIP_PREFIXES)
    )


def _build_geodetic_to_local_transformer(geo_reference: str):
    if CRS is None or Transformer is None:
        return None

    target_crs = CRS.from_user_input(_sanitize_geo_reference(geo_reference))
    return Transformer.from_crs(CRS.from_epsg(4326), target_crs, always_xy=True)


def _uses_default_carla_projection(geo_reference: str) -> bool:
    return (
        _extract_proj_string_parameter(geo_reference, "+proj=", "") == "tmerc"
        and math.isclose(_extract_proj_parameter(geo_reference, "+lat_0=", 0.0), 0.0)
        and math.isclose(_extract_proj_parameter(geo_reference, "+lon_0=", 0.0), 0.0)
        and math.isclose(_extract_proj_parameter(geo_reference, "+k=", 1.0), 1.0)
        and math.isclose(_extract_proj_parameter(geo_reference, "+x_0=", 0.0), 0.0)
        and math.isclose(_extract_proj_parameter(geo_reference, "+y_0=", 0.0), 0.0)
    )


class CoordinateTransformer:
    """This class can be used to transform coordinates between
    the x/y/z and GNSS reference frame"""

    la_ref = STD_LAT
    ln_ref = STD_LON
    h_ref = STD_H
    geo_reference = None
    _geodetic_to_local = None
    _use_projected_transform = False
    ref_set = False

    def __init__(self):
        pass

    @classmethod
    def configure_from_geo_reference(cls, geo_reference: str):
        cls.geo_reference = geo_reference.strip()
        cls.la_ref = _extract_proj_parameter(cls.geo_reference, "+lat_0=", STD_LAT)
        cls.ln_ref = _extract_proj_parameter(cls.geo_reference, "+lon_0=", STD_LON)
        cls.h_ref = _extract_proj_parameter(cls.geo_reference, "+h_0=", STD_H)
        cls._geodetic_to_local = _build_geodetic_to_local_transformer(cls.geo_reference)
        cls._use_projected_transform = (
            cls._geodetic_to_local is not None
            and not _uses_default_carla_projection(cls.geo_reference)
        )
        cls.ref_set = True

    def gnss_to_xyz(self, lat, lon, h):
        if self._use_projected_transform and self._geodetic_to_local is not None:
            return self.projected_geodetic_to_xyz(lat, lon, h)
        # Default CARLA maps still round-trip GNSS XY through the legacy transform
        # more accurately than through the advertised zeroed tmerc projection.
        x, y, _ = self.geodetic_to_enu(lat, lon, h)
        return x, y, float(h)

    def projected_geodetic_to_xyz(self, lat, lon, alt):
        """Project WGS84 coordinates into CARLA's local map frame.

        CARLA's projected northing is inverted relative to the localization frame,
        so the returned y coordinate keeps the existing localization convention.
        """
        x, y = self._geodetic_to_local.transform(lon, lat)
        # CARLA's GNSS altitude already matches the local map frame, so adding the
        # legacy map offset here double-counts elevation.
        return float(x), float(-y), float(alt)

    def geodetic_to_enu(self, lat, lon, alt):
        """
        Method from pylot project to calculate coordinates
        https://github.com/erdos-project/pylot/blob/master/pylot/utils.py#L470

        Also seems to match the internal carla api implementation
        https://github.com/carla-simulator/carla/blob/4a6622d15dc4f1c5247e470882dc639743e17e14/LibCarla/source/carla/geom/GeoLocation.cpp#L38

        With the special carla 0.9.14(leaderboard) version,
        this function provides an accurate conversion.
        With carla 0.9.16, the results are off by up to 2m in y...?

        Args:
            lat (float): latitude
            lon (float): longitude
            alt (float: altitude

        Returns:
            x, y, z: coordinates
        """

        scale = math.cos(self.la_ref * math.pi / 180.0)
        basex = scale * math.pi * a / 180.0 * self.ln_ref
        basey = scale * a * math.log(math.tan((90.0 + self.la_ref) * math.pi / 360.0))

        x = scale * math.pi * a / 180.0 * lon - basex
        y = scale * a * math.log(math.tan((90.0 + lat) * math.pi / 360.0)) - basey

        # Is not necessary in new version
        y *= -1
        # alt_offset is needed to keep the hight of the map in mind
        # right now we don't really use the altitude anyways
        return x, y, alt + alt_offset

    def pymap3d_geodetic_to_enu(self, lat: float, lon: float, alt: float):
        """Converts geodetic coordinates to enu (carla) coordinates

        Compared to the other implementations it uses
        pymap3d's functions for the conversion

        In theory this should be the most correct conversion,
        but in practive it seems to be even more off
        than the other geodetic_to_enu function. (up to 4m in y!)

        To use it across the project, just replace the call in gnss_to_xyz with this fn.

        Args:
            lat (float): latitude
            lon (float): longitude
            alt (float): altitude

        Returns:
            x, y, z: coordinates
        """
        # https://stackoverflow.com/a/62521868
        x, y, z = pymap3d.geodetic2enu(
            lat,
            lon,
            alt,
            self.la_ref,
            self.ln_ref,
            self.h_ref,
            deg=True,
            ell=pymap3d.ellipsoid.Ellipsoid.from_name("wgs84"),
        )
        return x, -y, z


def geodetic_to_ecef(lat, lon, h):
    # (lat, lon) in WSG-84 degrees
    # h in meters
    lamb = math.radians(lat)
    phi = math.radians(lon)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    sin_phi = math.sin(phi)
    cos_phi = math.cos(phi)

    x = (h + N) * cos_lambda * cos_phi
    y = (h + N) * cos_lambda * sin_phi
    z = (h + (1 - e_sq) * N) * sin_lambda

    return x, y, z


def ecef_to_enu(x, y, z, lat0, lon0, h0):
    lamb = math.radians(lat0)
    phi = math.radians(lon0)
    s = math.sin(lamb)
    N = a / math.sqrt(1 - e_sq * s * s)

    sin_lambda = math.sin(lamb)
    cos_lambda = math.cos(lamb)
    s_phi = math.sin(phi)
    c_phi = math.cos(phi)

    x0 = (h0 + N) * cos_lambda * c_phi
    y0 = (h0 + N) * cos_lambda * s_phi
    z0 = (h0 + (1 - e_sq) * N) * sin_lambda

    xd = x - x0
    yd = y - y0
    zd = z - z0

    xE = -s_phi * xd + c_phi * yd
    yN = -c_phi * sin_lambda * xd - sin_lambda * s_phi * yd + cos_lambda * zd
    zUp = cos_lambda * c_phi * xd + cos_lambda * s_phi * yd + sin_lambda * zd

    return xE, yN, zUp


def quat_to_heading(quaternion):
    """
    Converts a quaternion to the heading of the car in radians
    :param quaternion: quaternion of the car as a list [q.x, q.y, q.z, q.w]
                       where q is the quaternion
    :return: heading of the car in radians (float)
    """
    # Create a Rotation object from the quaternion
    rotation = Rotation.from_quat(quaternion)
    # Convert the Rotation object to a matrix
    rotation_matrix = rotation.as_matrix()
    # Calculate the angle around the z-axis (theta) from the matrix
    theta = np.arctan2(rotation_matrix[1, 0], rotation_matrix[0, 0])

    # arctan2 returns a theta so that:
    # ---------------------------------------------------------------
    # | 0 = x-axis | pi/2 = y-axis | pi = -x-axis | -pi/2 = -y-axis |
    # ---------------------------------------------------------------
    # heading is positive in counter clockwise rotations

    heading = -theta

    return heading

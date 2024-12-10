import rospy
import cython

from . import entity, map, shape, transform
from .map import Map

if cython.compiled:
    print("mapping_common is compiled!")
else:
    print("mapping_common is not compiled!")

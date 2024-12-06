cimport numpy as np

cdef class _Coord2():
    cdef np.ndarray _matrix

    cpdef float x(self)

cdef class Point2(_Coord2):
    pass

cdef class Vector2(_Coord2):
    pass

cdef class Transform2D:
    cdef np.ndarray _matrix
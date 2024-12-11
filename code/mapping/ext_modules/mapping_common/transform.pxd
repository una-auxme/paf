cimport numpy as np

cdef class _Coord2():
    cdef readonly np.ndarray _matrix

    cpdef double x(self)
    cpdef set_x(self, value: double)
    cpdef double y(self)
    cpdef void set_y(self, value: double)

cdef class Point2(_Coord2):
    pass

cdef class Vector2(_Coord2):
    pass

cdef class Transform2D:
    cdef readonly np.ndarray _matrix

    cpdef Vector2 translation(self)
cimport numpy as np

cdef class _Coord2():
    cdef readonly np.ndarray _matrix

    cpdef double x(self)
    cpdef set_x(self, double value)
    cpdef double y(self)
    cpdef void set_y(self, double value)

cdef class Point2(_Coord2):
    pass

cdef class Vector2(_Coord2):
    
    cpdef double length(self)
    cpdef Vector2 normalized(self)
    cpdef double angle_to(self, Vector2 other)

cdef class Transform2D:
    cdef readonly np.ndarray _matrix

    cpdef Vector2 translation(self)
    cpdef double rotation(self)

    cpdef Transform2D inverse(self)
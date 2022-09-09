# coding=utf-8

"""
Module implementing the Vector class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012-2018"
__credits__ = ["Morten Lind"]
__license__ = "GPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.dyndns.dk"
__status__ = "Production"


import numpy as np

from . import utils


class _UnitVectors(type):

    @property
    def e0(self):
        return Vector(1, 0, 0)
    ex = e0

    @property
    def e1(self):
        return Vector(0, 1, 0)
    ey = e1

    @property
    def e2(self):
        return Vector(0, 0, 1)
    ez = e2

    @property
    def unit_vectors(self):
        return [Vector.ex, Vector.ey, Vector.ez]


class Vector(object, metaclass=_UnitVectors):
    """A Vector is a 3D vector (member of R3) with standard Euclidian
    operations."""

    @classmethod
    def canCreateOn(cls, *arg):
        if type(arg) == cls:
            return True
        elif utils.is_sequence(arg):
            if len(arg) <= 3 and utils.is_num_types(arg):
                return True
            elif len(arg) == 1:
                return cls.canCreateOn(*arg[0])
            else:
                return False
        else:
            return False

    def __init__(self, *args, **kwargs):
        """Constructor for Vector. If optional keyword argument
        'position' evaluates to True, or is not given, the vector is
        represented as a position vector. Otherwise it is represented
        as a free vector."""
        if len(args) == 0:
            self._data = np.array([0, 0, 0], dtype=utils.flt)
        elif len(args) == 3 and utils.is_num_types(args):
            self._data = np.array(args, dtype=utils.flt)
        elif len(args) == 2 and utils.is_num_types(args):
            self._data = np.array((args[0], args[1], 0), dtype=utils.flt)
        elif len(args) == 1:
            arg = args[0]
            if utils.is_three_sequence(arg):
                self._data = np.array(arg, dtype=utils.flt)
            elif utils.is_sequence(arg) and len(arg) == 2:
                self._data = np.array((arg[0], arg[1], 0), dtype=utils.flt)
            elif type(arg) == Vector:
                self._data = arg.array
            elif hasattr(arg, "x") and hasattr(arg, "y") and hasattr(arg, "z"):
                self._data = np.array([arg.x, arg.y, arg.z])
            else:
                raise utils.Error(
                    ('__init__ : could not create vector on argument ' +
                     ': "{}" of type "{}"')
                    .format(str(args[0]), str(type(args[0]))))
        else:
            raise utils.Error('__init__ : could not create vector on ' +
                              'argument : "{}" of type "{}"'
                              .format(str(args[0]), str(type(args[0]))))
        self._is_position = kwargs.get('position', 1)

    def __copy__(self):
        """Copy method for creating a copy of this Vector."""
        return Vector(self)

    def __deepcopy__(self, memo):
        return self.__copy__()

    def copy(self, other=None):
        """Copy data from 'other' to self. If no argument given,
        i.e. 'other==None', return a copy of this Vector."""
        if other is None:
            return Vector(self)
        else:
            self._data[:] = other._data

    def __getattr__(self, name):
        if name == 'x':
            return self._data[0]
        elif name == 'y':
            return self._data[1]
        elif name == 'z':
            return self._data[2]
        else:
            raise AttributeError('Attribute "{}" not found in Vector'
                                 .format(name))

    def __setattr__(self, name, val):
        if name == 'x':
            self._data[0] = val
        elif name == 'y':
            self._data[1] = val
        elif name == 'z':
            self._data[2] = val
        # elif name == '_data':
        #     # Important for initialization? Or would
        #     # object.__setattr__ take care of it?
        #     self.__dict__[name] = val
        elif name == 'pos':
            if type(val) == Vector:
                self._data[:] = val.array
            elif utils.is_three_sequence(val):
                self._data[:] = np.array(val)
        else:
            object.__setattr__(self, name, val)

    # These pose some semantic problems with numpy array multiplication.
    # def __len__(self):
    #     return 3

    # def __iter__(self):
    #     return iter(self._data)

    def __getitem__(self, n):
        return self._data[n]

    def __setitem__(self, n, val):
        self._data[n] = val

    def __eq__(self, other):
        if type(other) == Vector:
            return np.allclose(self._data, other._data)
        else:
            return NotImplemented

    def __repr__(self):
        return ('<Vector: ({:.5f}, {:.5f}, {:.5f})>'
                .format(*self._data))

    def __str__(self):
        return self.__repr__()

    def get_is_position(self):
        """If the vector is a position vector, the default, then it
        transforms differently than a real vector.
        """
        return self._is_position

    is_position = property(get_is_position)

    def projection(self, v, normalize=True):
        """Get the projection vector of this vector onto a direction given by
        another vector, 'v'. If 'normalize' is True, the default, 'v'
        is first normalized. For performance reasons, normalization of
        'v' can be avoided if the caller assures that it is of unit
        length, and this can be flagged by setting normalize to False.
        """
        if normalize:
            v_hat = v.normalized
        else:
            v_hat = v
        return (self * v_hat) * v_hat

    def angle(self, other, minimal=False):
        """Return the angle (radians) to the 'other' vector. This is the
        absolute, positive angle unless minimal is True, in which case
        the minimal of the two angles between the vectors is returned.
        """
        costheta = (self * other) / (self.length * other.length)
        if costheta > 1:
            costheta = 1
        elif costheta < -1:
            costheta = -1
        ang = np.arccos(costheta)
        if minimal:
            return min(ang, np.pi-ang)
        else:
            return ang

    def signed_angle_to(self, other, ref_vec=None):
        """With default reference rotation vector as Z-axis (if
        'ref_vec' == None), compute the signed angle of rotation from
        self to 'other'.
        """
        theta = self.angle(other)
        xprod = self.cross(other)
        if ref_vec is not None:
            if xprod * ref_vec < 0:
                theta = -theta
        else:
            if xprod.z < 0:
                theta = -theta
        return theta

    def signed_angle(self, other, ref_vec=None):
        utils._deprecation_warning('signed_angle_to')
        return self.signed_angle_to(other, ref_vec=None)

    def get_length(self):
        """Return the Euclidean length."""
        return np.sqrt(self.length_squared)

    length = property(get_length)

    def get_length_squared(self):
        """Return the square of the standard Euclidean length."""
        return np.dot(self._data, self._data)

    length_squared = property(get_length_squared)

    def normalize(self):
        """In-place normalization of this Vector."""
        length = self.length
        if length != 1.0:
            self._data = self._data / length

    def get_normalized(self):
        """Return a normalized Vector with same direction as this
        one.
        """
        nv = Vector(self)
        nv.normalize()
        return nv

    normalized = property(get_normalized)

    def dist(self, other):
        """Compute euclidean distance between points given by self
        and 'other'."""
        return np.sqrt(self.dist_squared(other))

    def dist_squared(self, other):
        """Compute euclidean distance between points given by self
        and 'other'."""
        if type(other) == Vector:
            sub = np.subtract(self._data, other._data)
            return np.dot(sub, sub)
        else:
            return NotImplemented

    def get_cross_operator(self):
        """Return the cross product operator for this Vector. I.e. the
        skew-symmetric operator cross_op, such that cross_op * u == v
        x u, for any vector u."""
        cross_op = np.zeros((3, 3))
        cross_op[0, 1] = -self._data[2]
        cross_op[0, 2] = self._data[1]
        cross_op[1, 0] = self._data[2]
        cross_op[1, 2] = -self._data[0]
        cross_op[2, 0] = -self._data[1]
        cross_op[2, 1] = self._data[0]
        return cross_op

    cross_operator = property(get_cross_operator)

    def get_alt_cross_operator(self):
        """Return the cross product operator for this Vector. I.e. the
        skew-symmetric operator cross_op, such that cross_op * u == v
        x u, for any vector u."""
        cross_op = np.zeros((3, 3))
        # cross_op[0, 1] = -self._data[2]
        cross_op[0, 2] = self._data[1]
        cross_op[1, 0] = self._data[2]
        # cross_op[1, 2] = -self._data[0]
        # cross_op[2, 0] = -self._data[1]
        cross_op[2, 1] = self._data[0]
        return cross_op - cross_op.T

    alt_cross_operator = property(get_alt_cross_operator)

    def cross(self, other):
        """Return the cross product with 'other'."""
        return Vector(np.cross(self._data, other._data))

    def get_array(self):
        """Return a copy of the ndarray which is the fundamental data
        of the Vector."""
        return self._data.copy()

    def set_array(self, array, check=True):
        """Set the vector by three values in the iterable 'array'.
        """
        if check and len(array) != 3:
            raise Exception(
                self.__class__.__name__ +
                'Setting the value by the "array" property needs exactly'
                + ' three values. ({} were given)'.format(len(array)))
        self._data[:] = array

    array = property(get_array, set_array)

    def get_array_ref(self):
        """Return a reference to the (3,) ndarray, which is the
        fundamental data of the Orientation. Caution: Use this method
        only for optimization, since it eliminates copying, and be
        sure not to compromize the data.
        """
        return self._data

    array_ref = property(get_array_ref)

    def get_list(self):
        """Return the fundamental data of the Vector as a list."""
        return self._data.tolist()
    list = property(get_list)

    def get_matrix(self):
        """Property for getting a single-column np-matrix with the data
        from the vector."""
        return np.matrix(self._data).T

    matrix = property(get_matrix)

    def get_column(self):
        """Property for getting a single-column array with the data
        from the vector."""
        return self._data.reshape((3, 1))

    column = property(get_column)

    def __sub__(self, other):
        """Subtract another vector from this. The semantics regarding
        free and position vectors should be: If this is free, and
        other is a position, or opposite, the new should be
        position. If both are free or both are positions, the new
        should be free."""
        if type(other) == Vector:
            return Vector(self._data - other._data)
        elif type(other) == np.ndarray and other.shape == (3,):
            # Other is given as a triplet in ndarray
            return Vector(self._data - other)
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be multiplied
        #     return [self - o for o in other]
        else:
            return NotImplemented

    def __isub__(self, other):
        if type(other) == Vector:
            self._data -= other._data
        elif type(other) == np.ndarray and other.shape == (3,):
            # Other is given as a triplet in ndarray
            self._data -= other
        else:
            return NotImplemented
        return self

    def __mul__(self, other):
        """Multiplication with an 'other' Vector (inner product) or
        with a scalar."""
        if type(other) == Vector:
            return np.dot(self._data, other._data)
        elif utils.is_num_type(other):
            return Vector(self._data * other)
        elif type(other) == np.ndarray and other.shape == (3,):
            # Other is given as a triplet in ndarray
            return Vector(self._data * other)
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be multiplied
        #     # WARNING: v * [1,2,3] == [1*v, 2*v, 3*v] !
        #     return [self * o for o in other]
        else:
            return NotImplemented

    def __imul__(self, other):
        """In-place multiplication with a scalar, 'other'. """
        if utils.is_num_type(other):
            self._data *= other
        else:
            return NotImplemented
            # raise utils.Error('__imul__ : Could not multiply by non-number')
        return self

    def __rmul__(self, other):
        """Right multiplication with a scalar, 'other'. """
        if utils.is_num_type(other):
            return Vector(other * self._data)
        else:
            raise utils.Error('__rmul__ : Could not multiply by non-number')

    def __truediv__(self, other):
        """Division with a scalar, 'other'. """
        if utils.is_num_type(other):
            if np.isclose(other, 0.0):
                raise ZeroDivisionError('In division of vector by scalar')            
            return Vector(1.0 / other * self._data)
        else:
            raise utils.Error('__truediv__ : Could not divide by non-number')
    __div__ = __truediv__

    def __add__(self, other):
        """Return the sum of this and the 'other' vector."""
        if type(other) == Vector:
            return Vector(self._data + other._data)
        elif type(other) == np.ndarray and other.shape == (3,):
            # Other is given as a triplet in ndarray
            return Vector(self._data + other)
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be added
        #     return [self + o for o in other]
        else:
            return NotImplemented
            # raise utils.Error('__add__ : Could not add non-vector')

    def __iadd__(self, other):
        """In-place add the 'other' vector to this vector."""
        if type(other) == Vector:
            self._data += other._data
        elif type(other) == np.ndarray and other.shape == (3,):
            # Other is given as a triplet in ndarray
            self._data += other
        else:
            return NotImplemented
            # raise utils.Error('__iadd__ : Could not add non-vector')
        return self

    def __neg__(self):
        return Vector(-self._data)

    @classmethod
    def new_random_unit_vector(cls):
        """Generator for random vectors uniformly sampled on S2. Use the
        Muller's algorithm from "A note on a method for generating
        points uniformly on n-dimensional spheres", Communications of
        the ACM, Volume 2, Issue 4, April 1959, pp 19-20.
        """
        v = Vector(np.random.normal(size=3))
        v.normalize()
        return v





def random_unit_vector():
    """Generator for random vectors uniformly sampled on S2. Use the
    Muller's algorithm from "A note on a method for generating points
    uniformly on n-dimensional spheres". Communications of the
    ACM. Volume 2, Issue 4, April 1959, pp 19-20."""
    utils._deprecation_warning('Vector.new_random_unit_vector')
    v = Vector(np.random.normal(size=3))
    v.normalize()
    return v


def _test_construction():
    print((Vector.canCreateOn(1, 2, 3),
           Vector.canCreateOn((1, 2, 3)),
           Vector.canCreateOn(1, 2)))


def _test_signed_angle():
    v = Vector(1, 2, 3)
    u = Vector(3, 1, 2)
    print(v.signed_angle_to(u))
    return True


def _test_rops():
    """Test that rop on other is called when NotImplemented is
    returned from the Vector object."""
    class A(object):
        def __rmul__(self, other):
            print('rmul from A')
            return other * 2.0
    v = Vector(1, 2, 3)
    v_origin = v.copy()
    a = A()
    if v*a != 2*v:
        return False
    v *= a
    if v != 2 * v_origin:
        return False
    u = Vector(3, 1, 2)
    print(v.signed_angle_to(u))
    return True


def _test_projection():
    v0 = Vector([1, 1, 3])
    v1 = Vector([1, 1, 0])
    v0prj = v0.projection(v1)
    if (v0prj - v1).length > utils.eps:
        return False
    v0prj = v0.projection(v1.normalized, normalize=False)
    if (v0prj - v1).length > utils.eps:
        return False
    return True


# def _test_vectorized_operations():
#     # Test multiplication of a list
#     v = Vector(1, 0, 0)
#     vs = [Vector(1, 0, 0), Vector(0, 1, 0)]
#     rms = v * vs
#     assert(rms[0] == v * vs[0])
#     assert(rms[1] == v * vs[1])
#     ras = v + vs
#     assert(ras[0] == v + vs[0])
#     assert(ras[1] == v + vs[1])
#     rss = v + vs
#     assert(rss[0] == v + vs[0])
#     assert(rss[1] == v + vs[1])
    

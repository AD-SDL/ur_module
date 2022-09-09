# coding=utf-8

"""
Module implementing a 3D homogenous Transform class. The transform is
represented internally by associated orientation and a vector objects.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2012-2016"
__credits__ = ["Morten Lind"]
__license__ = "GPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.dyndns.dk"
__status__ = "Production"


import numpy as np

from . import utils
from .vector import Vector
from .orientation import Orientation


class Transform(object):
    """A Transform is a member of SE(3), represented as a homogenous
    transformation matrix. It uses an Orientation in member '_o'
    (accessible through 'orient') to represent the orientation part
    and a Vector in member '_v' (accessible through 'pos') to
    represent the position part.
    """

    # A set of acceptable multi-value types for entering data.
    __value_types = (np.ndarray, list, tuple)

    class Error(Exception):
        """Exception class."""
        def __init__(self, message):
            self.message = message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __create_on_sequence(self, arg):
        """Called from init when a single argument of sequence type was given
        the constructor.
        """
        # if len(arg) == 1 and utils.is_sequence(arg[0]):
        #     self.__createOnSequence(arg[0])
        if type(arg) in (tuple, list):
            self.__create_on_sequence(np.array(arg))
        elif type(arg) == np.ndarray and arg.shape in ((4, 4), (3, 4)):
            self._o = Orientation(arg[:3, :3])
            self._v = Vector(arg[:3, 3])
        elif type(arg) == np.ndarray and arg.shape == (6,):
            # Assume a pose vector of 3 position vector and 3 rotation
            # vector components
            self._v = Vector(arg[:3])
            self._o = Orientation(arg[3:])
        else:
            raise self.Error(
                'Could not create Transform on arguments : "' + str(arg) + '"')

    def __init__(self, *args):
        """A Transform is a homogeneous transform on SE(3), internally
        represented by an Orientation and a Vector. A Transform can be
        constructed on:

        * A Transform.

        * A numpy array, list or tuple of shape (4,4) or (3,4) giving
          direct data; as [orient | pos].

        * A --''-- of shape (6,) giving a pose vector; concatenated
          position and rotation vector.

        * Two --''--; the first for orientation and the second for
          position.

        * Four --''--; the first three for orientation and the fourth
          for position.

        * Twelve numbers, the first nine used for orientation and the
          last three for position.

        * An ordered pair of Orientation and Vector.
        """
        if len(args) == 0:
            self._v = Vector()
            self._o = Orientation()
        elif len(args) == 1:
            arg = args[0]
            if type(arg) == Transform or (
                    hasattr(arg, 'pos') and hasattr(arg, 'orient')):
                self._v = Vector(arg.pos)
                self._o = Orientation(arg.orient)
            else:
                self.__create_on_sequence(arg)
        elif len(args) == 2:
            self._o = Orientation(args[0])
            self._v = Vector(args[1])
        elif len(args) == 4:
            self._o = Orientation(args[:3])
            self._v = Vector(args[3])
        elif len(args) == 12:
            self._o = Orientation(args[:9])
            self._v = Vector(args[9:])
        else:
            raise self.Error(
                'Could not create Transform on arguments : ' +
                '"{}"'.format(str(args)))
        # Guard against reference to data.
        self._from_ov(self._o, self._v)

    def _from_ov(self, o, v):
        self._data = np.identity(4, dtype=utils.flt)
        # First take over the data from Orientation and Vector
        self._data[:3, :3] = o._data
        self._data[:3, 3] = v._data
        # Then share data with Orientation and Vector.
        self._o._data = self._data[:3, :3]
        self._v._data = self._data[:3, 3]

    def get_pos(self):
        """Return a reference (Beware!) to the position object."""
        return self._v

    def set_pos(self, new_pos):
        """Set the position."""
        if type(new_pos) in self.__value_types:
            self._data[:3, 3] = new_pos
        elif type(new_pos) == Vector:
            self._data[:3, 3] = new_pos._data
        else:
            raise self.Error('Trying to set "pos" by an object of ' +
                             'type "{}". '.format(str(type(new_pos))) +
                             'Needs tuple, list, ndarray, or Vector.')

    pos = property(get_pos, set_pos)

    def get_orient(self):
        """Return a reference (Beware!) to the orientation object."""
        return self._o

    def set_orient(self, new_orient):
        """Set the orientation."""
        if type(new_orient) in self.__value_types:
            self._data[:3, :3] = new_orient
        elif type(new_orient) == Orientation:
            self._data[:3, :3] = new_orient._data
        else:
            raise self.Error('Trying to set "orient" by an object of ' +
                             'type "{}". '.format(str(type(new_orient))) +
                             'Needs tuple, list, ndarray, or Orientation.')

    orient = property(get_orient, set_orient)

    def __copy__(self):
        """Copy method for creating a (deep) copy of this
        Transform.
        """
        return Transform(self)

    def __deepcopy__(self, memo):
        return self.__copy__()

    def copy(self, other=None):
        """Copy data from 'other' to self. If no argument given,
        i.e. 'other==None', return a copy of this Transform.
        """
        if other is None:
            return Transform(self)
        else:
            self._data[:, :] = other._data

    def __repr__(self):
        return ('<Transform:\n{}\n{}\n>'
                .format(repr(self.orient), repr(self.pos)))

    def __str__(self):
        return self.__repr__()

    def __eq__(self, other):
        if type(other) == Transform:
            return np.sum((self._data - other._data) ** 2) < utils.eps
        else:
            return NotImplemented

    def from_xyp(self, vec_x, vec_y, origo):
        """Make this transform correspond to the orientation given by the
        given 'vec_x' and 'vec_y' directions and translation given by
        'origo'.
        """
        self._o.from_xy(vec_x, vec_y)
        self._v = origo
        self._from_ov(self._o, self._v)

    def from_xzp(self, vec_x, vec_z, origo):
        """Make this transform correspond to the orientation given by the
        given 'vec_x' and 'vec_z' directions and translation given by
        'p'.
        """
        self._o.from_xz(vec_x, vec_z)
        self._v = origo
        self._from_ov(self._o, self._v)

    def from_yzp(self, vec_y, vec_z, origo):
        """Make this transform correspond to the orientation given by the
        given 'vec_y' and 'vec_z' directions and translation given by
        'p'.
        """
        self._o.from_yz(vec_y, vec_z)
        self._v = origo
        self._from_ov(self._o, self._v)

    def dist_squared(self, other):
        """Return the square of the metric distance, as the unweighted sum of
        linear and angular distance, to the 'other' transform. Note
        that the units and scale among linear and angular
        representations matters heavily.
        """
        return self._v.dist_squared(other._v) + self._o.ang_dist(other._o) ** 2

    def dist(self, other):
        """Return the metric distance, as unweighted combined linear and
        angular distance, to the 'other' transform. Note that the
        units and scale among linear and angular representations
        matters heavily.
        """
        return np.sqrt(self.dist_squared(other))

    def get_inverse(self):
        """Return an inverse of this Transform."""
        return Transform(np.linalg.inv(self._data))

    inverse = property(get_inverse)

    def invert(self):
        """In-place invert this Transform."""
        self._data[:, :] = np.linalg.inv(self._data)

    def __mul__(self, other):
        """Multiplication of self with another Transform or operate on a
        Vector given by 'other'. 'other' may be a Transform, a Vector,
        an 3-ndarray, or an 3xN-ndarray.
        """
        if type(other) == Transform:
            return Transform(np.dot(self._data, other._data))
        elif type(other) == Vector:
            v = np.ones(4)
            v[:3] = other._data
            return Vector(np.dot(self._data, v)[:3])
        elif (type(other) == np.ndarray and
              len(other.shape) in (1, 2) and
              other.shape[0] == 3):
            return (np.matmul(self._o._data, other).T + self._v._data).T
        # elif utils.is_sequence(other):
        #     # Assume a sequence of objects that may be multiplied
        #     return [self * o for o in other]
        else:
            return NotImplemented

    def get_pose_vector(self):
        """Get the transform in pose vector representation '(x, y, z, rx, ry,
        rz)'.
        """
        return np.append(self._v._data, self._o.rotation_vector)

    pose_vector = property(get_pose_vector)

    def get_structured_array(self):
        """Return a tuple pair of an 3x3 orientation array and position as
        3-array.
        """
        return (self._data[:3, :3], self._data[:3, 3])

    structured_array = property(get_structured_array)

    def get_structued_list(self):
        """Return a list with separate orientation and position in list
        form.
        """
        return [self._data[:3, :3].tolist(), self._data[:3, 3].tolist()]

    structued_list = property(get_structued_list)

    def get_matrix(self):
        """Property for getting a (4,4) np-matrix with the data from the
        transform.
        """
        return np.matrix(self._data)

    matrix = property(get_matrix)

    def get_array(self):
        """Return a copy of the (4,4) ndarray which is the fundamental
        data of the Transform. Caution: Use this method only for
        optimization, since it eliminates copying, and be sure not to
        compromize the data.
        """
        return self._data.copy()

    array = property(get_array)

    def get_array_ref(self):
        """Return a reference to the (4,4) ndarray, which is the
        fundamental data of the transform.
        """
        return self._data

    array_ref = property(get_array_ref)

    def get_list(self):
        """Return the fundamental data of the Transform as a list."""
        return self._data.tolist()
    list = property(get_list)

    @classmethod
    def new_from_xyp(self, vec_x, vec_y, origo):
        """Create a transform corresponding to the orientation given by the
        given 'vec_x' and 'vec_y' directions and translation given by
        'origo'.
        """
        t = Transform()
        t.from_xyp(vec_x, vec_y, origo)
        return t

    @classmethod
    def new_from_xzp(self, vec_x, vec_z, origo):
        """Create a transform corresponding to the orientation given by the
        given 'vec_x' and 'vec_z' directions and translation given by
        'origo'.
        """
        t = Transform()
        t.from_xzp(vec_x, vec_z, origo)
        return t

    @classmethod
    def new_from_yzp(self, vec_y, vec_z, origo):
        """Create a transform corresponding to the orientation given by the
        given 'vec_y' and 'vec_z' directions and translation given by
        'origo'.
        """
        t = Transform()
        t.from_yzp(vec_y, vec_z, origo)
        return t

    @classmethod
    def new_from_point_sets(self, ApTs: np.ndarray, BpTs: np.ndarray):
        """Create a transform inferred from the same set of minimum three
        points, observed in two reference systems, 'A' and 'B'. Points
        are stored as row vector, hence they are denoted 'XpTs'. The
        resulting transform is denoted AB, which transforms from
        reference B to referende A.
        """
        # Calculate centered point set
        ApTc = ApTs.mean(axis=0)
        ApTcs = ApTs - ApTc
        BpTc = BpTs.mean(axis=0)
        BpTcs = BpTs - BpTc
        U, S, VT = np.linalg.svd(ApTcs.T @ BpTcs)
        # Orientation of B in A reference
        AoB = U @ VT
        # Correct for reflection
        if  np.linalg.det(AoB) < 0:
            AoB = U @ np.diag([1.0, 1.0, -1]) @ VT
        # Calculate origo of B in A reference
        ApB = ApTc - AoB @ BpTc
        return Transform(AoB, ApB)


def _test():
    cy = Vector(1, 1, 0)
    cx = Vector(2, 3, 0)
    cz = Vector.e2
    p = Vector(1, 2, 3)
    t = Transform.new_from_xzp(cx, cz, p)
    t = Transform.new_from_yzp(cy, cz, p)
    print(t * cx)
    it = t.inverse
    print(t * it)


def _test_transf_vectors():
    px = Vector.e0
    py = Vector.e1
    pz = Vector.e2
    pp = Vector(1, 2, 3)
    t = Transform(Orientation.new_rot_z(np.pi), Vector(1, 0, 0))
    tpx = t * px
    assert(tpx == Vector(0, 0, 0))
    assert(np.all(t * px.array == tpx.array))
    tpy = t * py
    assert(tpy == Vector(1, -1, 0))
    tpz = t * pz
    assert(tpz == Vector(1, 0, 1))
    tpp = t * pp
    assert(tpp == Vector(0, -2, 3))
    pstack = np.vstack([px.array, py.array, pz.array, pp.array]).T
    tpstack = t * pstack
    stacktp = np.vstack([tpx.array, tpy.array, tpz.array, tpp.array]).T
    assert(np.all(tpstack == stacktp))


def _test_from_point_sets():
    BpTs = np.array([[1, 0, 1],
                     [1, 1, 0],
                     [0, 0, 1],
                     #[2, 0, 0]
                    ])
    AB_nom = Transform(Orientation.new_euler([0.1, 0.5, 1.2]),
                       Vector(1,2,3))
    ApTs = (AB_nom.orient.array @ BpTs.T).T + AB_nom.pos.array 
    AB = Transform.new_from_point_sets(ApTs, BpTs)
    print(f'Orientation error: {AB.orient.ang_dist(AB_nom.orient)} rad')
    print(f'Position error: {AB.pos.dist(AB_nom.pos)} [length units]')

   

# def _test_vectorized_multiplication():
#     # Test multiplication of a list
#     t = Transform(Orientation.new_rot_z(np.pi/2), Vector(1, 0, 0))
#     vs = [Vector(1, 0, 0), Vector(0, 1, 0)]
#     rs = t * vs
#     assert(rs[0] == t * vs[0])
#     assert(rs[1] == t * vs[1])
    

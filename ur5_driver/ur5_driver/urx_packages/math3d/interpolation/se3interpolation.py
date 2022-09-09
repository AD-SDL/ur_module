# coding=utf-8

"""
Module implementing the SE(3) interpolator class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2009-2018"
__credits__ = ["Morten Lind"]
__license__ = "GPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.dyndns.dk"
__status__ = "Production"

from ..transform import Transform
from .. import utils
from .so3interpolation import SO3Interpolation
from .r3interpolation import R3Interpolation


class SE3Interpolation(SO3Interpolation, R3Interpolation):
    """A class for object representing a linear interpolation in task
    space, SE(3), between two points. Interpolation is done from one
    configuration, trf0, to another, trf1. trf0 and trf1 can be given
    as Transform objects."""

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = 'SE3Interpolation Error: ' + message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self, trf0, trf1, shortest=True):
        """Initialise an SE(3) interpolation from transform 'trf0' to
        transform 'trf1'. If 'shortest' is True, the shortest rotation
        path is chosen, if False, the long rotation is used, and if
        None it is indeterminate, given by the Versor objects being
        constructed from the transforms.
        """
        self._trf0 = trf0
        self._trf1 = trf1
        SO3Interpolation.__init__(self, self._trf0.orient,
                                  self._trf1.orient, shortest)
        R3Interpolation.__init__(self, self._trf0.pos, self._trf1.pos)

    def __call__(self, time, checkrange=True):
        """Class callable method for giving the transform at time
        'time'; in [0,1]."""
        if checkrange:
            time = utils.flt(time)
            if time < 0.0 or time > 1.0:
                raise self.Error('"time" must be number in [0.0 ; 1.0]. ' +
                                 'It was {}.'.format(time))
        return Transform(self.orient(time, False), self.pos(time, False))


TaskLinearInterpolation = SE3Interpolation
EuclideanInterpolation = SE3Interpolation


def _test():
    """Simple test function."""
    global o0, o1, tint, p0, p1
    from math3d.orientation import Orientation
    from math3d.vector import Vector
    from math import pi
    p0 = Vector([0, 1, 0])
    p1 = Vector([1, 0, 1])
    o0 = Orientation()
    o0.set_to_x_rotation(pi / 2)
    o1 = Orientation()
    o1.set_to_z_rotation(pi / 2)
    tint = SE3Interpolation(Transform(o0, p0), Transform(o1, p1))

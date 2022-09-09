# coding=utf-8

"""
Module implementing the SO(3) interpolator class; Slerp.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2009-2018"
__credits__ = ["Morten Lind"]
__license__ = "GPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.dyndns.dk"
__status__ = "Production"

import numpy as np

from ..orientation import Orientation
from ..quaternion import Versor, UnitQuaternion
from .. import utils


class SO3Interpolation(object):
    """A SLERP interpolator class in SO(3)."""

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = 'SO3Interpolation Error: ' + message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self, start, end, shortest=True):
        """Initialise an SO(3) interpolation from orientation 'start' to
        orientation 'end'. If 'shortest' is True the shortest rotation
        path is chosen, if False the long rotation is used, and if
        None it is indeterminate, given by the Versor objects
        constructed over 'start' and 'end'.
        """
        self._qstart = Versor(start)
        self._qend = Versor(end)
        self._qstart.normalize()
        self._qend.normalize()
        if shortest is not None:
            if shortest and (self._qstart.dist(self._qend) >
                             self._qstart.dist(-self._qend)):
                self._qend = -self._qend
            elif not shortest and (self._qstart.dist(self._qend) <
                                   self._qstart.dist(-self._qend)):
                self._qend = -self._qend
        self._qstartconj = self._qstart.conjugated.normalized
        self._qstartconjqend = (self._qstartconj * self._qend).normalized

    def __call__(self, t):
        return self.versor(t)

    def versor(self, time, checkrange=True):
        """Return the versor of the slerp at 'time'; in [0,1]."""
        if checkrange:
            time = utils.flt(time)
            if time < 0.0 or time > 1.0:
                raise self.Error('"time" must be number in [0.0 ; 1.0]. ' +
                                 'I was {}'.format(time))
        return self._qstart * (self._qstartconjqend) ** time

    def orient(self, time, checkrange=True):
        """Return the orientation in the slerp at 'time'; in [0,1]. """
        return self.versor(time, checkrange).orientation


SLERP = SO3Interpolation
OrientationInterpolation = SO3Interpolation


def _test():
    """Simple test function."""
    global o, o1, q, q1, osl, qsl
    from math import pi
    o = Orientation()
    o.set_to_x_rotation(pi / 2)
    o1 = Orientation()
    o1.set_to_z_rotation(pi / 2)
    q = Versor(o)
    q1 = Versor(o1)
    qsl = SO3Interpolation(q, q1)
    osl = SO3Interpolation(o, o1)

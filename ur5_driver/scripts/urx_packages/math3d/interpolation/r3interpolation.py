# coding=utf-8

"""
Module implementing the R^3 interpolator class.
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2009-2015"
__credits__ = ["Morten Lind"]
__license__ = "GPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.dyndns.dk"
__status__ = "Production"

import numpy as np

from ..vector import Vector
from .. import utils

class R3Interpolation(object):
    """Simple linear position interpolation in R^3."""

    class Error(Exception):
        """Exception class."""

        def __init__(self, message):
            self.message = 'R3Interpolation Error: ' + message
            Exception.__init__(self, self.message)

        def __repr__(self):
            return self.message

    def __init__(self, p0, p1):
        """Make a position interpolation between 'p0' and' p1'. 'p0' and 'p1'
        must be suitable data for creation of Vectors."""
        self._p0 = Vector(p0)
        self._p1 = Vector(p1)
        self._displ = self._p1 - self._p0

    def __call__(self, time, checkrange=True):
        """Class callable method for directly invoking the pos
        method. 'time' must be given in the interval [0;1]."""
        return self.pos(time, checkrange)

    def pos(self, time, checkrange=True):
        """Called to get the interpolated position at 'time'."""
        if checkrange:
            time = utils.flt(time)
            if time < 0.0 or time > 1.0:
                raise self.Error('"time" must be number in [0.0 ; 1.0]. ' +
                                 'It was {}.'.format(time))
        return self._p0 + time * self._displ


PositionInterpolation = R3Interpolation


def _test():
    """Simple test function."""
    global p0, p1, pint
    p0 = [0, 1, 0]
    p1 = [1, 0, 1]
    pint = R3Interpolation(p0, p1)


if __name__ == '__main__':
    import readline
    readline.parse_and_bind("tab: complete")

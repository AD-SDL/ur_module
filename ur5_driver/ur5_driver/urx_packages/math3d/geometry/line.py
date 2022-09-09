# coding=utf-8

"""
Module for line class
"""

__author__ = "Morten Lind"
__copyright__ = "Morten Lind 2016-2019"
__credits__ = ["Morten Lind"]
__license__ = "GPLv3"
__maintainer__ = "Morten Lind"
__email__ = "morten@lind.fairuse.org"
__status__ = "Development"


import math3d as m3d
import numpy as np

from . import plane

class Line(object):
    """A line class."""

    def __init__(self, **kwargs):
        """Supported, named constructor arguments:

        * 'point_direction' or 'pd_pair': An ordered pair of vectors
          representing a point on the line and the line's
          direction. The direction does not have to be a unit vector.

        * 'point', 'direction': Separate vectors for point on line and
          direction of line in named arguments. The direction does not
          have to be a unit vector.

        * 'point0', 'point1': Two points defining the line, in named
         arguments.

        * 'points': A set of at least two points is used for
        PCA-identification of the direction of the line, and where the
        line point is chosen as the average position.
        """

        if 'point_direction' in kwargs:
            self._p, self._d = [m3d.Vector(e)
                                for e in kwargs['point_direction']]
        elif 'pd_pair' in kwargs:
            self._p, self._d = [m3d.Vector(e)
                                for e in kwargs['pd_pair']]
        elif 'point' in kwargs and 'direction' in kwargs:
            self._p = m3d.Vector(kwargs['point'])
            self._d = m3d.Vector(kwargs['direction'])
        elif 'point0' in kwargs and 'point1' in kwargs:
            self._p = m3d.Vector(kwargs['point0'])
            self._d = m3d.Vector(kwargs['point1']) - self._p
        elif 'points' in kwargs:
            self.fit_points(kwargs['points'])
        else:
            raise Exception(
                'Can not create Line object on given arguments: "{}"'
                .format(kwargs))
        # Create the unit direction vector
        self._ud = self._d.normalized

    def __repr__(self):
        return '<Line: p=[{:.3f}, {:.3f}, {:.3f}] ud=[{:.3f}, {:.3f}, {:.3f}]>'.format(
            *tuple(self._p), *tuple(self._ud))

    @property
    def point(self):
        return m3d.Vector(self._p)

    @property
    def direction(self):
        return m3d.Vector(self._d)

    @property
    def unit_direction(self):
        return m3d.Vector(self._ud)

    def fit_points(self, points):
        """Compute the line from a set of points. 'points'
        must be an array of row position vectors, such that
        points[i] is a position vector."""
        points = np.array(points)
        centre = np.sum(points, axis=0)/len(points)
        eigen = np.linalg.eig(np.cov(points.T))
        max_ev_i = np.where(eigen[0] == max(eigen[0]))[0][0]
        direction = eigen[1].T[max_ev_i]
        self._p = m3d.Vector(centre)
        self._d = self._ud = m3d.Vector(direction)

    @classmethod
    def new_fitted_points(cls, points):
        return cls(points=points)

    def projected_point(self, p):
        """Return the projection of 'p' onto this line."""
        p2p = (self._p - p)
        return p + p2p - (self._ud * p2p) * self._ud

    def projected_line(self, line):
        """Return the projection of 'line' onto this line. I.e. return the
        point in this line, closest to 'line'. If the lines are
        parallel, the origin point of this line is returned. The
        'use_plane' argument flags for using the Plane class
        implementation for implementation simplicity.  See
        https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points.
        """
        # Check if the lines are parallel
        if (1 - self._ud * line._ud) < 10 * m3d.utils.eps:
            return self.point
        # https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
        p1 = self._p
        p2 = line._p
        d1 = self._ud
        d2 = line._ud
        n2 = d2.cross(d1.cross(d2))
        return p1 + ((p2-p1) * n2 / (d1 * n2)) * d1

    def nearest_points(self, line):
        """Return the pair of closest points on this line and 'line'. The
        first of the returned points is on this line. See
        https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
        """ 
           # Check if the lines are parallel
        if (1 - self._ud * line._ud) < 10 * m3d.utils.eps:
            return self.point
        # https://en.wikipedia.org/wiki/Skew_lines#Nearest_Points
        p1 = self._p
        p2 = line._p
        d1 = self._ud
        d2 = line._ud
        n1 = d1.cross(d2.cross(d1))
        n2 = d2.cross(d1.cross(d2))
        return (p1 + ((p2-p1) * n2 / (d1 * n2)) * d1,
                p2 + ((p1-p2) * n1 / (d2 * n1)) * d2)


def _test():
    # Simple test of projected line.
    l1 = Line(point=(0, 1, 1), direction=(0, 0.1, 1))
    l2 = Line(point=(1, 1, 0.1), direction=(0, 1, 0))
    pl = l1.projected_line(l2)
    # print('Projected line: {}'.format(str(pl)))
    assert((pl-l1.projected_point(pl))
           .length < 10 * m3d.utils.eps)
    # Test line fitting.
    fl = Line.new_fitted_points([[1, 1, 1],
                                 [2, 2, 2],
                                 [3, 3, 3]])
    # print('Fitted line: {}'.format(str(fl)))
    assert(fl.point == m3d.Vector(2, 2, 2))
    assert(1.0 - np.abs(fl.direction * m3d.Vector(1, 1, 1).normalized)
           < 10 * m3d.utils.eps)

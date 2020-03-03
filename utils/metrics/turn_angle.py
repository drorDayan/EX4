from arr2_epec_seg_ex import *
from utils.metrics.distance import distance_squared
from math import pi, tan, acos, sqrt


def calc_angle_2d(p0, p1, p2):
    return acos(((p1[0] - p0[0]) * (p2[0] - p1[0]) +
                 (p1[1] - p0[1]) * (p2[1] - p1[1])).to_double() /
                sqrt(distance_squared(p1, p0).to_double()) *
                sqrt(distance_squared(p2, p1).to_double())) * 180 / pi


def calc_angles(p0, p1, p2):
    assert(None not in (p1, p2) and p1.dimension() == p2.dimension())
    if p0 is None:
        return [FT(0)] * p1.dimension()
    assert(p0.dimension() == p1.dimension())
    return [FT(calc_angle_2d(Point_d(2, [p0[2 * i], p0[2 * i + 1]]),
                             Point_d(2, [p1[2 * i], p1[2 * i + 1]]),
                             Point_d(2, [p2[2 * i], p2[2 * i + 1]])))
            for i in range(int(p1.dimension() / 2))]

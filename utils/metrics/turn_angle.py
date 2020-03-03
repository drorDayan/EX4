from arr2_epec_seg_ex import *
# TODO implement this, if time permits.
from utils.metrics.distance import distance_squared
from math import tan, acos


def calc_angle_transformed(p0, p1, p2):
    if None in (p0, p1, p2):
        return FT(0)
    temp = (sum([(p1[i] - p0[i]) * (p2[i] - p1[i]) for i in range(p0.dimension())], FT(0)) /
            distance_squared(p1, p0) * distance_squared(p2, p1)).to_double()
    return FT(tan(acos(temp) / 2))

from arr2_epec_seg_ex import FT, Point_d
from math import sqrt
from utils.metrics.distance import distance_squared


def steer(robot_num, near, rand, eta):
    dist = FT(sqrt(distance_squared(near, rand).to_double()))
    if dist < eta:
        return rand, False
    else:
        return Point_d(2 * robot_num, [near[i] + (rand[i] - near[i]) * eta / dist for i in range(2 * robot_num)]), True

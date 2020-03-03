from utils.misc import between
from math import sqrt
from arr2_epec_seg_ex import *


def min_dist_between_moving_robots_l_inf(s1, s2, t1, t2):
    # 2DO Enumerate over all Gods, Goddesses, Kami, etc. and for each one pray that this code works.
    sx = s1[0] - s2[0]
    sy = s1[1] - s2[1]

    tx = t1[0] - t2[0]
    ty = t1[1] - t2[1]

    dx = tx - sx
    dy = ty - sy

    m = dy / dx
    n = (dx * sy - dy * sx) / dx
    candidates = [max(abs(sx), abs(sy)), max(abs(tx), abs(ty))]
    if m != 1:
        z = n / (1 - m)
        if between(sx, z, tx):
            candidates.append(abs(z))
    if m != -1:
        z = n / (-1 - m)
        if between(sx, z, tx):
            candidates.append(abs(z))
    return min(candidates)


def min_dist_between_moving_robots(s1, s2, t1, t2):
    sx = s1[0] - s2[0]
    sy = s1[1] - s2[1]

    tx = t1[0] - t2[0]
    ty = t1[1] - t2[1]

    dx = tx - sx
    dy = ty - sy
    n = (dx * sy - dy * sx) / dx
    m2_1 = (dy / dx) ** 2 + 1

    if between(sy, n / m2_1, ty):
        return abs(n) / sqrt(m2_1)
    else:
        return min(sqrt(sx ** 2 + sy ** 2),
                   sqrt(tx ** 2 + ty ** 2))


def min_inter_robot_distance(start_point, target_point):
    robot_num = int(start_point.dimension()/2)
    min_dist = None
    for i in range(robot_num):
        s1 = (start_point[2 * i].to_double(), start_point[2 * i + 1].to_double())
        t1 = (target_point[2 * i].to_double(), target_point[2 * i + 1].to_double())
        for j in range(i + 1, robot_num):
            s2 = (start_point[2 * j].to_double(), start_point[2 * j + 1].to_double())
            t2 = (target_point[2 * j].to_double(), target_point[2 * j + 1].to_double())

            new_min_dist = min_dist_between_moving_robots(s1, s2, t1, t2)
            min_dist = new_min_dist if min_dist is None else min(min_dist, new_min_dist)

    return FT(min_dist)
